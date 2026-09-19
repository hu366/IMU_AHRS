"""Offline VQF vs Madgwick on one recorded 6DoF CSV (task 4.2).

PC only. Does not run in imu_viewer. Device quaternion column is optional
and only used to check the firmware vqf-c port against official VQF.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .allan_analysis import _payload_line, parse_comment_meta
from .madgwick_offline import DEFAULT_BETA, run_madgwick

G_MPS2 = 9.80665
DEFAULT_TAU_ACC = 3.0
DEFAULT_STILL_GYR = 0.05  # rad/s, ~2.9 dps
DEFAULT_STILL_ACC_G = 0.15
DEFAULT_RECOVER_DEG = 2.0
CSV_HEADER = "t_us,gx,gy,gz,ax,ay,az,qw,qx,qy,qz"
QUAT_COLS = ("qw", "qx", "qy", "qz")


def median_dt_s(t_us: np.ndarray) -> float:
    if t_us.size < 2:
        raise ValueError("need at least 2 timestamps to get dt from t_us")
    dt = np.diff(t_us.astype(np.float64)) / 1e6
    dt = dt[dt > 0]
    if dt.size == 0:
        raise ValueError("t_us is not increasing")
    return float(np.median(dt))


def firmware_gyr_ts(series: AhrsSeries, dt_nom: float) -> float:
    """VQF-C on device is inited with 1/nominal_fs, not the UART dump median dt.

    Dump printf can stretch the loop (e.g. 100 Hz nominal, ~77 Hz logged).
    Algorithm-quality replay still uses t_us; the port check must use the
    same fixed Ts the firmware filter used.
    """
    raw = series.meta.get("nominal_fs")
    if raw is None or str(raw).strip() == "":
        return dt_nom
    try:
        fs = float(raw)
    except ValueError:
        return dt_nom
    if fs <= 0.0:
        return dt_nom
    return 1.0 / fs


def sample_dt_s(t_us: np.ndarray, dt_nom: float) -> np.ndarray:
    """Per-sample dt from t_us, clamped like firmware imu_task (0.5x..2x nominal)."""
    n = t_us.size
    dts = np.full(n, dt_nom, dtype=float)
    if n < 2:
        return dts
    raw = np.diff(t_us.astype(np.float64)) / 1e6
    raw = np.concatenate([[dt_nom], raw])
    lo, hi = 0.5 * dt_nom, 2.0 * dt_nom
    dts = np.clip(raw, lo, hi)
    dts[0] = dt_nom
    return dts


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n = np.maximum(n, 1e-12)
    return q / n


def quat_angle_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = quat_normalize(a)
    b = quat_normalize(b)
    dot = np.abs(np.sum(a * b, axis=-1))
    dot = np.clip(dot, 0.0, 1.0)
    return 2.0 * np.degrees(np.arccos(dot))


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Active rotate v by unit Hamilton q (body -> world)."""
    q = quat_normalize(q)
    w = q[:, 0]
    x = q[:, 1]
    y = q[:, 2]
    z = q[:, 3]
    vx, vy, vz = v[:, 0], v[:, 1], v[:, 2]
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    out = np.empty_like(v, dtype=float)
    out[:, 0] = vx + w * tx + (y * tz - z * ty)
    out[:, 1] = vy + w * ty + (z * tx - x * tz)
    out[:, 2] = vz + w * tz + (x * ty - y * tx)
    return out


def tilt_err_deg(q: np.ndarray, acc_g: np.ndarray) -> np.ndarray:
    """Angle between rotated accel and earth +Z (specific force / up)."""
    mag = np.linalg.norm(acc_g, axis=-1, keepdims=True)
    mag = np.maximum(mag, 1e-12)
    acc_n = acc_g / mag
    world = quat_rotate(q, acc_n)
    c = np.clip(world[:, 2], -1.0, 1.0)
    return np.degrees(np.arccos(c))


def yaw_rad(q: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def unwrap_deg(rad: np.ndarray) -> np.ndarray:
    return np.degrees(np.unwrap(rad))


def still_mask(gyr: np.ndarray, acc_g: np.ndarray, gyr_th: float, acc_th: float) -> np.ndarray:
    gyr_mag = np.linalg.norm(gyr, axis=-1)
    acc_mag = np.linalg.norm(acc_g, axis=-1)
    return (gyr_mag < gyr_th) & (np.abs(acc_mag - 1.0) < acc_th)


def _longest_true_run(mask: np.ndarray) -> slice:
    if mask.size == 0 or not np.any(mask):
        return slice(0, 0)
    best_s, best_e, cur_s = 0, 0, None
    for i, flag in enumerate(mask):
        if flag and cur_s is None:
            cur_s = i
        elif not flag and cur_s is not None:
            if i - cur_s > best_e - best_s:
                best_s, best_e = cur_s, i
            cur_s = None
    if cur_s is not None and mask.size - cur_s > best_e - best_s:
        best_s, best_e = cur_s, mask.size
    return slice(best_s, best_e)


def _delta(vqf: float, madg: float) -> float:
    if not math.isfinite(madg) or madg == 0.0:
        return float("nan")
    return (vqf - madg) / madg


def run_vqf_official(
    gyr: np.ndarray,
    acc_g: np.ndarray,
    gyr_ts: float,
    tau_acc: float = DEFAULT_TAU_ACC,
    motion_bias: bool = True,
    rest_bias: bool = True,
) -> np.ndarray:
    from vqf import VQF

    acc = np.asarray(acc_g, dtype=np.float64) * G_MPS2
    gyr = np.asarray(gyr, dtype=np.float64)
    vqf = VQF(float(gyr_ts), float(gyr_ts), -1.0)
    vqf.setMagDistRejectionEnabled(False)
    vqf.setMotionBiasEstEnabled(bool(motion_bias))
    vqf.setRestBiasEstEnabled(bool(rest_bias))
    vqf.setTauAcc(float(tau_acc))
    out = vqf.updateBatch(gyr, acc)
    return np.asarray(out["quat6D"], dtype=float)


@dataclass
class AhrsSeries:
    t_us: np.ndarray
    gyr: np.ndarray
    acc_g: np.ndarray
    q_esp: np.ndarray | None
    meta: dict[str, str]
    path: Path | None = None

    @property
    def n(self) -> int:
        return int(self.t_us.size)


def load_ahrs_text(text: str, source_path: Path | None = None) -> AhrsSeries:
    meta = parse_comment_meta(text)
    t_list: list[int] = []
    gyr_list: list[list[float]] = []
    acc_list: list[list[float]] = []
    q_list: list[list[float]] = []
    have_q = False
    for raw in text.splitlines():
        line = _payload_line(raw)
        if not line or line.startswith("#"):
            continue
        low = line.lower().replace(" ", "")
        if low.startswith("t_us,"):
            have_q = all(c in low.split(",") for c in QUAT_COLS)
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 7:
            continue
        try:
            t_us = int(float(parts[0]))
            nums = [float(x) for x in parts[1:7]]
        except ValueError:
            continue
        t_list.append(t_us)
        gyr_list.append(nums[0:3])
        acc_list.append(nums[3:6])
        if have_q and len(parts) >= 11:
            try:
                q_list.append([float(x) for x in parts[7:11]])
            except ValueError:
                q_list.append([float("nan")] * 4)
        else:
            q_list.append([float("nan")] * 4)

    if not t_list:
        raise ValueError("no AHRS CSV rows (need t_us,gx,gy,gz,ax,ay,az)")

    t_us = np.asarray(t_list, dtype=np.int64)
    gyr = np.asarray(gyr_list, dtype=float)
    acc = np.asarray(acc_list, dtype=float)
    q_esp_arr = np.asarray(q_list, dtype=float)
    q_esp = None if not np.isfinite(q_esp_arr).all() else q_esp_arr
    if q_esp is None and np.isfinite(q_esp_arr).any():
        q_esp = q_esp_arr
        q_esp[~np.isfinite(q_esp).all(axis=1)] = np.nan

    unit = meta.get("accel_unit", "g").lower()
    if unit in ("m/s2", "m/s^2", "mps2"):
        acc = acc / G_MPS2
        meta["accel_unit"] = "g"
    return AhrsSeries(t_us=t_us, gyr=gyr, acc_g=acc, q_esp=q_esp, meta=meta, path=source_path)


def load_ahrs_file(path: Path) -> AhrsSeries:
    return load_ahrs_text(path.read_text(encoding="utf-8", errors="replace"), path)


def synthesize_ahrs(
    kind: str = "still",
    *,
    fs_hz: float = 100.0,
    seed: int = 0,
) -> AhrsSeries:
    """Synthetic 6DoF in hand frame, accel in g, gyro rad/s. No device quat."""
    rng = np.random.default_rng(seed)
    dt = 1.0 / fs_hz
    if kind == "still":
        n = int(5.0 * fs_hz)
        t = np.arange(n, dtype=np.int64) * int(round(1e6 / fs_hz))
        gyr = rng.normal(0.0, 0.002, size=(n, 3))
        acc = np.column_stack(
            [
                rng.normal(0.0, 0.01, size=n),
                rng.normal(0.0, 0.01, size=n),
                rng.normal(1.0, 0.01, size=n),
            ]
        )
    elif kind == "bias_still":
        n = int(20.0 * fs_hz)
        t = np.arange(n, dtype=np.int64) * int(round(1e6 / fs_hz))
        gyr = rng.normal(0.0, 0.002, size=(n, 3))
        gyr[:, 2] += 0.01  # residual yaw bias ~0.57 dps
        acc = np.column_stack(
            [
                rng.normal(0.0, 0.01, size=n),
                rng.normal(0.0, 0.01, size=n),
                rng.normal(1.0, 0.01, size=n),
            ]
        )
    elif kind == "xrot":
        n_still = int(1.0 * fs_hz)
        n_rot = int(round(0.5 * math.pi / dt))  # 90 deg at 1 rad/s
        n_end = int(1.0 * fs_hz)
        n = n_still + n_rot + n_end
        t = np.arange(n, dtype=np.int64) * int(round(1e6 / fs_hz))
        gyr = rng.normal(0.0, 0.001, size=(n, 3))
        acc = np.zeros((n, 3))
        theta = 0.0
        for i in range(n):
            if n_still <= i < n_still + n_rot:
                gyr[i, 0] += 1.0
                theta += 1.0 * dt
            acc[i, 0] = rng.normal(0.0, 0.005)
            acc[i, 1] = math.sin(theta) + rng.normal(0.0, 0.005)
            acc[i, 2] = math.cos(theta) + rng.normal(0.0, 0.005)
    elif kind == "shake":
        n_a = int(2.0 * fs_hz)
        n_m = int(1.0 * fs_hz)
        n_b = int(3.0 * fs_hz)
        n = n_a + n_m + n_b
        t = np.arange(n, dtype=np.int64) * int(round(1e6 / fs_hz))
        gyr = rng.normal(0.0, 0.002, size=(n, 3))
        acc = np.column_stack(
            [
                rng.normal(0.0, 0.01, size=n),
                rng.normal(0.0, 0.01, size=n),
                rng.normal(1.0, 0.01, size=n),
            ]
        )
        gyr[n_a : n_a + n_m] = rng.normal(0.0, 1.5, size=(n_m, 3))
        acc[n_a : n_a + n_m] = rng.normal(0.0, 0.4, size=(n_m, 3))
        acc[n_a : n_a + n_m, 2] += 1.0
    else:
        raise ValueError(f"unknown synthetic kind {kind!r}")

    meta = {
        "source": "synthetic",
        "kind": kind,
        "accel_unit": "g",
        "gyro_unit": "rad/s",
        "frame": "hand",
        "bias_subtracted": "1",
        "algo": "synthetic",
    }
    return AhrsSeries(t_us=t, gyr=gyr, acc_g=acc, q_esp=None, meta=meta)


def ahrs_to_csv_text(series: AhrsSeries) -> str:
    lines = [
        "# ahrs_csv source={source} algo={algo} accel_unit=g gyro_unit=rad/s "
        "frame=hand bias_subtracted={bias}".format(
            source=series.meta.get("source", "unknown"),
            algo=series.meta.get("algo", "unknown"),
            bias=series.meta.get("bias_subtracted", "1"),
        )
    ]
    extra = series.meta.get("kind")
    if extra:
        lines.append(f"# kind={extra}")
    lines.append(CSV_HEADER)
    q = series.q_esp
    for i in range(series.n):
        row = [
            str(int(series.t_us[i])),
            f"{series.gyr[i, 0]:.7g}",
            f"{series.gyr[i, 1]:.7g}",
            f"{series.gyr[i, 2]:.7g}",
            f"{series.acc_g[i, 0]:.5g}",
            f"{series.acc_g[i, 1]:.5g}",
            f"{series.acc_g[i, 2]:.5g}",
        ]
        if q is not None and np.isfinite(q[i]).all():
            row.extend(f"{q[i, k]:.7g}" for k in range(4))
        else:
            row.extend(["nan"] * 4)
        lines.append(",".join(row))
    return "\n".join(lines) + "\n"


def compare_series(
    series: AhrsSeries,
    *,
    beta: float = DEFAULT_BETA,
    tau_acc: float = DEFAULT_TAU_ACC,
    still_gyr: float = DEFAULT_STILL_GYR,
    still_acc: float = DEFAULT_STILL_ACC_G,
    recover_deg: float = DEFAULT_RECOVER_DEG,
) -> dict[str, float | str]:
    dt_nom = median_dt_s(series.t_us)
    dts = sample_dt_s(series.t_us, dt_nom)
    ts_fw = firmware_gyr_ts(series, dt_nom)
    q_madg = run_madgwick(series.gyr, series.acc_g, dts, beta=beta)
    q_vqf = run_vqf_official(series.gyr, series.acc_g, dt_nom, tau_acc=tau_acc)
    if abs(ts_fw - dt_nom) > 1e-9:
        q_vqf_port = run_vqf_official(series.gyr, series.acc_g, ts_fw, tau_acc=tau_acc)
    else:
        q_vqf_port = q_vqf

    still = still_mask(series.gyr, series.acc_g, still_gyr, still_acc)
    run = _longest_true_run(still)
    t_s = (series.t_us.astype(np.float64) - float(series.t_us[0])) / 1e6

    def still_tilt_std(q: np.ndarray) -> float:
        if run.stop - run.start < 10:
            return float("nan")
        err = tilt_err_deg(q[run], series.acc_g[run])
        return float(np.std(err))

    def yaw_drift(q: np.ndarray) -> float:
        if run.stop - run.start < max(10, int(round(1.0 / dt_nom))):
            return float("nan")
        yaw = unwrap_deg(yaw_rad(q[run]))
        tt = t_s[run]
        span = tt[-1] - tt[0]
        if span < 1.0:
            return float("nan")
        coef = np.polyfit(tt, yaw, 1)
        return float(coef[0] * 60.0)  # deg/min

    def static_tilt(q: np.ndarray) -> float:
        if run.stop - run.start < 10:
            return float("nan")
        return float(np.mean(tilt_err_deg(q[run], series.acc_g[run])))

    def jitter(q: np.ndarray) -> float:
        if run.stop - run.start < 12:
            return float("nan")
        d = quat_angle_deg(q[run.start : run.stop - 1], q[run.start + 1 : run.stop])
        return float(np.sqrt(np.mean(d * d)))

    def recover(q: np.ndarray) -> float:
        moving = ~still
        if not np.any(moving) or not np.any(still):
            return float("nan")
        last_move = int(np.max(np.nonzero(moving)[0]))
        after = slice(last_move + 1, None)
        if after.start >= series.n:
            return float("nan")
        err = tilt_err_deg(q[after], series.acc_g[after])
        ok = np.where(err < recover_deg)[0]
        if ok.size == 0:
            return float("nan")
        i = after.start + int(ok[0])
        return float((series.t_us[i] - series.t_us[last_move]) / 1e6)

    still_m = still_tilt_std(q_madg)
    still_v = still_tilt_std(q_vqf)
    yaw_m = yaw_drift(q_madg)
    yaw_v = yaw_drift(q_vqf)
    tilt_m = static_tilt(q_madg)
    tilt_v = static_tilt(q_vqf)
    rec_m = recover(q_madg)
    rec_v = recover(q_vqf)
    jit_m = jitter(q_madg)
    jit_v = jitter(q_vqf)

    port_mean = float("nan")
    port_p95 = float("nan")
    port_used_conj = 0.0
    if series.q_esp is not None:
        qe = series.q_esp.copy()
        good = np.isfinite(qe).all(axis=1)
        if np.count_nonzero(good) > 10:
            mask = good & still if np.count_nonzero(good & still) > 10 else good
            err = quat_angle_deg(qe[mask], q_vqf_port[mask])
            qe_c = qe.copy()
            qe_c[:, 1:] *= -1.0
            err_c = quat_angle_deg(qe_c[mask], q_vqf_port[mask])
            if np.nanmean(err_c) + 5.0 < np.nanmean(err):
                err = err_c
                port_used_conj = 1.0
            port_mean = float(np.nanmean(err))
            port_p95 = float(np.nanpercentile(err, 95))

    return {
        "n": float(series.n),
        "fs_hz": 1.0 / dt_nom,
        "still_n": float(run.stop - run.start),
        "still_tilt_std_deg_madgwick": still_m,
        "still_tilt_std_deg_vqf": still_v,
        "still_tilt_std_deg_delta": _delta(still_v, still_m),
        "yaw_drift_deg_per_min_madgwick": yaw_m,
        "yaw_drift_deg_per_min_vqf": yaw_v,
        "yaw_drift_deg_per_min_delta": _delta(abs(yaw_v), abs(yaw_m)),
        "static_tilt_err_deg_madgwick": tilt_m,
        "static_tilt_err_deg_vqf": tilt_v,
        "static_tilt_err_deg_delta": _delta(tilt_v, tilt_m),
        "recover_s_madgwick": rec_m,
        "recover_s_vqf": rec_v,
        "recover_s_delta": _delta(rec_v, rec_m),
        "quat_jitter_rms_deg_madgwick": jit_m,
        "quat_jitter_rms_deg_vqf": jit_v,
        "quat_jitter_rms_deg_delta": _delta(jit_v, jit_m),
        "port_err_mean_deg": port_mean,
        "port_err_p95_deg": port_p95,
        "port_used_conjugate": port_used_conj,
        "port_ts_s": ts_fw,
        "source": series.meta.get("source", ""),
        "algo_recorded": series.meta.get("algo", ""),
        "q_madg": q_madg,
        "q_vqf": q_vqf,
        "still_mask": still,
        "dt_nom": dt_nom,
    }


def _fmt(x: object) -> str:
    if isinstance(x, str):
        return x
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return str(x)
    if not math.isfinite(xf):
        return "n/a"
    return f"{xf:.4g}"


def metrics_table(metrics: dict[str, object]) -> str:
    rows = [
        ("metric", "madgwick", "vqf", "delta"),
        (
            "still_tilt_std_deg",
            metrics["still_tilt_std_deg_madgwick"],
            metrics["still_tilt_std_deg_vqf"],
            metrics["still_tilt_std_deg_delta"],
        ),
        (
            "yaw_drift_deg_per_min",
            metrics["yaw_drift_deg_per_min_madgwick"],
            metrics["yaw_drift_deg_per_min_vqf"],
            metrics["yaw_drift_deg_per_min_delta"],
        ),
        (
            "static_tilt_err_deg",
            metrics["static_tilt_err_deg_madgwick"],
            metrics["static_tilt_err_deg_vqf"],
            metrics["static_tilt_err_deg_delta"],
        ),
        (
            "recover_s",
            metrics["recover_s_madgwick"],
            metrics["recover_s_vqf"],
            metrics["recover_s_delta"],
        ),
        (
            "quat_jitter_rms_deg",
            metrics["quat_jitter_rms_deg_madgwick"],
            metrics["quat_jitter_rms_deg_vqf"],
            metrics["quat_jitter_rms_deg_delta"],
        ),
    ]
    lines = []
    for row in rows:
        lines.append(f"{row[0]:<24} { _fmt(row[1]):<12} { _fmt(row[2]):<12} { _fmt(row[3])}")
    lines.append(
        f"{'port_err_mean_deg':<24} {'(device)':<12} {_fmt(metrics['port_err_mean_deg']):<12} "
        f"p95={_fmt(metrics['port_err_p95_deg'])}"
    )
    lines.append(f"{'fs_hz':<24} {_fmt(metrics['fs_hz'])}  n={_fmt(metrics['n'])}  still_n={_fmt(metrics['still_n'])}")
    return "\n".join(lines) + "\n"


def metrics_csv(metrics: dict[str, object]) -> str:
    names = [
        "still_tilt_std_deg",
        "yaw_drift_deg_per_min",
        "static_tilt_err_deg",
        "recover_s",
        "quat_jitter_rms_deg",
    ]
    lines = ["metric,madgwick,vqf,delta"]
    for name in names:
        lines.append(
            ",".join(
                [
                    name,
                    _fmt(metrics[f"{name}_madgwick"]),
                    _fmt(metrics[f"{name}_vqf"]),
                    _fmt(metrics[f"{name}_delta"]),
                ]
            )
        )
    lines.append(
        f"port_err_mean_deg,n/a,{_fmt(metrics['port_err_mean_deg'])},n/a"
    )
    lines.append(f"port_err_p95_deg,n/a,{_fmt(metrics['port_err_p95_deg'])},n/a")
    lines.append(f"fs_hz,{_fmt(metrics['fs_hz'])},n={_fmt(metrics['n'])},still_n={_fmt(metrics['still_n'])}")
    return "\n".join(lines) + "\n"


def write_conclusion(metrics: dict[str, object], path: Path) -> None:
    port = metrics["port_err_mean_deg"]
    lines = [
        "VQF vs Madgwick (same 6DoF CSV, PC offline)",
        f"source={metrics.get('source')!s} recorded_algo={metrics.get('algo_recorded')!s}",
        f"fs={_fmt(metrics['fs_hz'])} Hz  n={_fmt(metrics['n'])}",
        "",
        metrics_table(metrics),
        "delta = (vqf - madgwick) / madgwick. Smaller tilt/yaw/jitter is better.",
        "6D cannot remove yaw around gravity; yaw_drift is relative, not heading.",
        "port_err is firmware q vs official VQF (not the algorithm-quality row).",
        "port replay uses firmware nominal Ts (header nominal_fs); quality replay uses t_us.",
    ]
    if isinstance(port, float) and math.isfinite(port):
        if port < 0.5:
            lines.append(f"port check: mean {port:.3f} deg < 0.5 deg (ok).")
        else:
            lines.append(f"port check: mean {port:.3f} deg >= 0.5 deg — check units/dt/conjugate.")
        if float(metrics["port_used_conjugate"]) > 0:
            lines.append("port check used conjugate of device q (convention mismatch).")
        ts_fw = metrics.get("port_ts_s")
        dt_nom = metrics.get("dt_nom")
        if isinstance(ts_fw, float) and isinstance(dt_nom, float) and abs(ts_fw - dt_nom) > 1e-4:
            lines.append(
                f"dump loop was slower than nominal: median dt={dt_nom*1e3:.2f} ms, "
                f"port Ts={ts_fw*1e3:.2f} ms (printf can stall the 100 Hz task)."
            )
    else:
        lines.append("port check: no device quaternion column in this CSV.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Replay one AHRS CSV through Madgwick and official VQF 6D"
    )
    p.add_argument("--input", type=Path, help="ahrs CSV (from capture_ahrs). Omit with --synthetic")
    p.add_argument("--out", type=Path, default=Path("analysis/out"))
    p.add_argument(
        "--synthetic",
        choices=["still", "bias_still", "xrot", "shake"],
        help="run on generated 6DoF instead of a file",
    )
    p.add_argument("--beta", type=float, default=DEFAULT_BETA)
    p.add_argument("--tau-acc", type=float, default=DEFAULT_TAU_ACC)
    p.add_argument("--write-csv", type=Path, help="also write the synthetic/input copy")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.synthetic:
        series = synthesize_ahrs(args.synthetic)
    elif args.input is not None:
        path = args.input.expanduser()
        if not path.is_file():
            print(f"error: input not found: {path}", file=sys.stderr)
            return 2
        series = load_ahrs_file(path)
    else:
        print("error: pass --input CSV or --synthetic still|bias_still|xrot|shake", file=sys.stderr)
        return 2

    out_dir = args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.write_csv is not None:
        args.write_csv.parent.mkdir(parents=True, exist_ok=True)
        args.write_csv.write_text(ahrs_to_csv_text(series), encoding="utf-8")

    metrics = compare_series(series, beta=args.beta, tau_acc=args.tau_acc)
    table = metrics_table(metrics)
    csv_path = out_dir / "vqf_vs_madgwick.csv"
    txt_path = out_dir / "vqf_compare.txt"
    csv_path.write_text(metrics_csv(metrics), encoding="utf-8")
    write_conclusion(metrics, txt_path)
    print(table, end="")
    print(f"wrote {csv_path}")
    print(f"wrote {txt_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
