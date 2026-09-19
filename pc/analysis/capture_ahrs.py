"""UART monitor that saves the firmware AHRS CSV dump (task 4.2).

After APP_LOG_AHRS_CSV=1 and flash, run this script. It writes
analysis/data/ahrs.csv when the dump finishes. Do not also run idf.py monitor.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from pathlib import Path

from .allan_analysis import _payload_line
from .capture_still import _hard_reset, _list_ports
from .vqf_compare import CSV_HEADER

DUMP_DONE_MARK = "ahrs CSV dump done"
DUMP_START_MARK = "UART AHRS CSV dump"
DEFAULT_OUT = Path("analysis/data/ahrs.csv")
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_S = 240.0
DEFAULT_MIN_ROWS = 200


class AhrsCsvCollector:
    """Pull AHRS CSV rows (IMU + optional device quat) out of mixed IDF text."""

    def __init__(self) -> None:
        self.comments: list[str] = []
        self.rows: list[str] = []
        self.started = False
        self.done = False
        self.saw_header = False

    def feed(self, raw: str) -> None:
        if self.done:
            return
        line = _payload_line(raw)
        raw_stripped = raw.strip()
        if DUMP_DONE_MARK in raw_stripped or DUMP_DONE_MARK in line:
            self.done = True
            return
        if not line:
            return
        if line.startswith("#") and ("ahrs_csv" in line or "bias_subtracted" in line or "algo=" in line):
            if line not in self.comments:
                self.comments.append(line)
            self.started = True
            return
        low = line.lower().replace(" ", "")
        if low.startswith("t_us,") and "gx" in low and "qw" in low:
            self.saw_header = True
            self.started = True
            return
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 7:
            try:
                int(float(parts[0]))
                [float(x) for x in parts[1:7]]
            except ValueError:
                pass
            else:
                self.started = True
                self.saw_header = True
                self.rows.append(line)
                return
        if DUMP_START_MARK in line:
            self.started = True

    def to_text(self) -> str:
        lines: list[str] = []
        if self.comments:
            lines.extend(self.comments)
        else:
            lines.append(
                "# ahrs_csv source=esp32-c3 algo=unknown accel_unit=g "
                "gyro_unit=rad/s frame=hand bias_subtracted=1"
            )
        lines.append(CSV_HEADER)
        lines.extend(self.rows)
        return "\n".join(lines) + "\n"

    @property
    def n(self) -> int:
        return len(self.rows)


def extract_ahrs_log(text: str) -> AhrsCsvCollector:
    col = AhrsCsvCollector()
    for raw in text.splitlines():
        col.feed(raw)
    return col


def _write_ahrs_log(path: Path, collector: AhrsCsvCollector) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(collector.to_text(), encoding="utf-8")


def capture_serial(
    port: str,
    out_path: Path,
    *,
    baud: int = DEFAULT_BAUD,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    min_rows: int = DEFAULT_MIN_ROWS,
    reset: bool = True,
    echo: bool = True,
    stay_open: bool = False,
) -> int:
    try:
        import serial
    except ImportError:
        print("error: pyserial is required. pip install -r analysis/requirements.txt", file=sys.stderr)
        return 2

    collector = AhrsCsvCollector()
    deadline = time.monotonic() + timeout_s
    saved = False
    rc = 1

    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
    except serial.SerialException as exc:
        ports = ", ".join(_list_ports()) or "(none found)"
        print(f"error: cannot open {port}: {exc}", file=sys.stderr)
        print(f"available ports: {ports}", file=sys.stderr)
        print("同一个 COM 口不能再开 idf.py monitor。关掉监视后再运行本脚本。", file=sys.stderr)
        return 2

    def maybe_save() -> None:
        nonlocal saved, rc
        if saved or collector.n == 0:
            return
        _write_ahrs_log(out_path, collector)
        saved = True
        status = "complete" if collector.done else "incomplete"
        print(f"wrote {out_path}  rows={collector.n}  dump={status}")
        if collector.done and collector.n >= min_rows:
            rc = 0
        elif collector.n < min_rows:
            print(
                f"warning: only {collector.n} rows (want >= {min_rows}). "
                "Need APP_LOG_AHRS_CSV=1 and the full dump window.",
                file=sys.stderr,
            )

    with ser:
        print(f"monitor {port} @ {baud}  -> {out_path}")
        print("零偏结束后会打 AHRS CSV。按 6.1 口令做动作。Ctrl+C 结束。")
        if reset:
            _hard_reset(ser)
            ser.reset_input_buffer()
        print("若没有输出，按板上 RESET。")

        buf = b""
        try:
            while True:
                if not stay_open and time.monotonic() >= deadline:
                    break
                if collector.done and not stay_open:
                    break
                chunk = ser.read(1024)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw_b, buf = buf.split(b"\n", 1)
                    raw = raw_b.decode("utf-8", errors="replace").rstrip("\r")
                    if echo and raw:
                        print(raw)
                    was_done = collector.done
                    collector.feed(raw)
                    if collector.done and not was_done:
                        maybe_save()
        except KeyboardInterrupt:
            print("\n监视结束")

    if not saved:
        maybe_save()
    if collector.n == 0:
        print(
            "error: 没有采到 AHRS CSV。确认 APP_LOG_AHRS_CSV=1 并已烧录。",
            file=sys.stderr,
        )
        return 1
    return rc


def run_monitor_window(
    *,
    port: str | None,
    out_path: Path,
    baud: int,
    reset: bool,
    min_rows: int,
    analyze: bool,
    analyze_out: Path,
) -> int:
    import tkinter as tk
    from tkinter import messagebox, ttk

    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("error: pyserial is required. pip install -r analysis/requirements.txt", file=sys.stderr)
        return 2

    out_path = out_path.resolve() if not out_path.is_absolute() else out_path
    events: queue.Queue[tuple] = queue.Queue()
    stop = threading.Event()
    reader_holder: list[threading.Thread] = []
    ser_holder: list[object] = []
    collector = AhrsCsvCollector()
    saved = {"ok": False}
    result = {"rc": 0}

    root = tk.Tk()
    root.title("IMU 串口监视 — ahrs.csv")
    root.geometry("920x560")

    top = ttk.Frame(root, padding=8)
    top.pack(fill=tk.X)
    ttk.Label(top, text="串口").pack(side=tk.LEFT)
    port_var = tk.StringVar(value=port or "")
    port_box = ttk.Combobox(top, textvariable=port_var, width=16)

    def refresh_ports() -> None:
        names = [p.device for p in list_ports.comports()]
        port_box["values"] = names
        if not port_var.get() and len(names) == 1:
            port_var.set(names[0])

    refresh_ports()
    port_box.pack(side=tk.LEFT, padx=4)
    ttk.Button(top, text="刷新", command=refresh_ports).pack(side=tk.LEFT)
    ttk.Label(top, text="波特率").pack(side=tk.LEFT, padx=(12, 0))
    baud_var = tk.StringVar(value=str(baud))
    ttk.Entry(top, textvariable=baud_var, width=8).pack(side=tk.LEFT, padx=4)

    status_var = tk.StringVar(value="点「开始监视」。零偏后会录 AHRS CSV（约 120 s）。")
    ttk.Label(root, textvariable=status_var, padding=8).pack(fill=tk.X)

    text = tk.Text(root, wrap=tk.NONE, font=("Consolas", 10))
    yscroll = ttk.Scrollbar(root, orient=tk.VERTICAL, command=text.yview)
    text.configure(yscrollcommand=yscroll.set)
    yscroll.pack(side=tk.RIGHT, fill=tk.Y)
    text.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

    def append_line(line: str) -> None:
        text.insert(tk.END, line + "\n")
        text.see(tk.END)

    def close_serial() -> None:
        stop.set()
        if ser_holder:
            try:
                ser_holder[0].close()  # type: ignore[union-attr]
            except Exception:
                pass
            ser_holder.clear()
        if reader_holder:
            reader_holder[0].join(timeout=1.0)
            reader_holder.clear()

    def reader_loop(ser: object) -> None:
        buf = b""
        try:
            while not stop.is_set():
                try:
                    chunk = ser.read(1024)  # type: ignore[union-attr]
                except Exception as exc:
                    events.put(("error", str(exc)))
                    break
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw_b, buf = buf.split(b"\n", 1)
                    raw = raw_b.decode("utf-8", errors="replace").rstrip("\r")
                    events.put(("line", raw))
                    was_done = collector.done
                    collector.feed(raw)
                    if collector.done and not was_done:
                        _write_ahrs_log(out_path, collector)
                        saved["ok"] = True
                        events.put(("saved", collector.n))
        finally:
            events.put(("stopped", None))

    def start_monitor() -> None:
        chosen = port_var.get().strip()
        if not chosen:
            messagebox.showerror("串口", "先选 COM 口")
            return
        try:
            baud_n = int(baud_var.get().strip())
        except ValueError:
            messagebox.showerror("波特率", "波特率必须是数字，默认 115200")
            return
        close_serial()
        stop.clear()
        collector.comments.clear()
        collector.rows.clear()
        collector.started = False
        collector.done = False
        collector.saw_header = False
        saved["ok"] = False
        try:
            ser = serial.Serial(port=chosen, baudrate=baud_n, timeout=0.2)
        except serial.SerialException as exc:
            messagebox.showerror(
                "打不开串口",
                f"{exc}\n\n不要同时开 idf.py monitor。关掉后再点开始。",
            )
            return
        ser_holder.append(ser)
        if reset:
            try:
                _hard_reset(ser)
                ser.reset_input_buffer()
            except Exception:
                pass
        append_line(f"=== monitor {chosen} @ {baud_n} ===")
        append_line("零偏完成后开始录 AHRS CSV。按任务 6.1 口令做动作。")
        append_line("若没有输出，按板上 RESET。")
        status_var.set(f"监视中 {chosen} … 等待 AHRS CSV")
        start_btn.configure(state=tk.DISABLED)
        t = threading.Thread(target=reader_loop, args=(ser,), daemon=True)
        reader_holder.append(t)
        t.start()

    def on_saved(n: int) -> None:
        status_var.set(f"已写入 {out_path}  （{n} 行）。")
        append_line(f"=== wrote {out_path}  rows={n} ===")
        if n < min_rows:
            result["rc"] = 1
            messagebox.showwarning("太短", f"只采到 {n} 行。确认 APP_LOG_AHRS_CSV=1 且等到 dump done。")
            return
        result["rc"] = 0
        if analyze:
            try:
                from .vqf_compare import main as compare_main

                rc = compare_main(["--input", str(out_path), "--out", str(analyze_out)])
                result["rc"] = rc
                append_line(f"=== VQF 对比结束 rc={rc} -> {analyze_out} ===")
            except Exception as exc:
                append_line(f"=== VQF 对比失败: {exc} ===")
                result["rc"] = 1

    def poll_queue() -> None:
        try:
            while True:
                kind, payload = events.get_nowait()
                if kind == "line" and payload:
                    append_line(str(payload))
                    if collector.n and not collector.done:
                        status_var.set(f"正在记录 AHRS CSV … {collector.n} 行")
                elif kind == "saved":
                    on_saved(int(payload))
                elif kind == "error":
                    status_var.set(f"串口错误: {payload}")
                    start_btn.configure(state=tk.NORMAL)
                elif kind == "stopped":
                    start_btn.configure(state=tk.NORMAL)
                    if not saved["ok"]:
                        if collector.n:
                            _write_ahrs_log(out_path, collector)
                            status_var.set(f"监视结束，已保存不完整记录 {collector.n} 行 -> {out_path}")
                            result["rc"] = 1
                        else:
                            status_var.set("监视结束，没有采到 AHRS CSV。确认 APP_LOG_AHRS_CSV=1 并已烧录。")
                            result["rc"] = 1
        except queue.Empty:
            pass
        try:
            root.after(50, poll_queue)
        except tk.TclError:
            pass

    start_btn = ttk.Button(top, text="开始监视", command=start_monitor)
    start_btn.pack(side=tk.LEFT, padx=12)

    def on_close() -> None:
        close_serial()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(50, poll_queue)
    if port_var.get().strip():
        root.after(200, start_monitor)
    root.mainloop()
    return int(result["rc"])


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="打开串口监视，把 AHRS CSV 写成 ahrs.csv")
    p.add_argument("--port", help="串口，例如 COM5。不填则在窗口里选")
    p.add_argument("--out", type=Path, default=DEFAULT_OUT, help=f"ahrs.csv 路径（默认 {DEFAULT_OUT}）")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="仅 --console：等待 dump 的秒数")
    p.add_argument("--min-rows", type=int, default=DEFAULT_MIN_ROWS)
    p.add_argument("--no-reset", action="store_true", help="不翻转 RTS/DTR 复位")
    p.add_argument("--quiet", action="store_true", help="--console 时不回显")
    p.add_argument("--console", action="store_true", help="不用窗口，在当前终端监视")
    p.add_argument("--from-log", type=Path, help="从已有 monitor 文本抽取 CSV，不打开串口")
    p.add_argument("--analyze", action="store_true", help="写完 ahrs.csv 后立刻跑 vqf_compare")
    p.add_argument("--analyze-out", type=Path, default=Path("analysis/out"))
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    if args.from_log is not None:
        path = args.from_log.expanduser()
        if not path.is_file():
            print(f"error: log not found: {path}", file=sys.stderr)
            return 2
        collector = extract_ahrs_log(path.read_text(encoding="utf-8", errors="replace"))
        if collector.n == 0:
            print("error: no AHRS CSV rows in that log", file=sys.stderr)
            return 1
        _write_ahrs_log(args.out, collector)
        print(f"wrote {args.out}  rows={collector.n}  dump={'complete' if collector.done else 'incomplete'}")
        rc = 0 if collector.n >= args.min_rows else 1
        if rc == 0 and args.analyze:
            from .vqf_compare import main as compare_main

            rc = compare_main(["--input", str(args.out), "--out", str(args.analyze_out)])
        return rc

    if args.console:
        if not args.port:
            ports = ", ".join(_list_ports()) or "(none found)"
            print("error: --console 需要 --port", file=sys.stderr)
            print(f"available ports: {ports}", file=sys.stderr)
            return 2
        rc = capture_serial(
            args.port,
            args.out,
            baud=args.baud,
            timeout_s=args.timeout,
            min_rows=args.min_rows,
            reset=not args.no_reset,
            echo=not args.quiet,
            stay_open=False,
        )
        if rc == 0 and args.analyze:
            from .vqf_compare import main as compare_main

            rc = compare_main(["--input", str(args.out), "--out", str(args.analyze_out)])
        return rc

    return run_monitor_window(
        port=args.port,
        out_path=args.out,
        baud=args.baud,
        reset=not args.no_reset,
        min_rows=args.min_rows,
        analyze=args.analyze,
        analyze_out=args.analyze_out,
    )


if __name__ == "__main__":
    raise SystemExit(main())
