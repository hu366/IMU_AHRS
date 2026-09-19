"""Offline Allan-variance analysis of still IMU CSV (task 3.3).

PC only: no BLE, no VPython, no firmware AHRS path.
dt/fs come from device t_us, never from the PC wall clock.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
ESP_LOG_RE = re.compile(r"^[IWE] \(\d+\)\s+\S+:\s+(.*)$")
COMMENT_KV_RE = re.compile(r"([A-Za-z_][\w]*)\s*=\s*([^\s,]+)")
FLOAT_RE = r"[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?"
DATA_RE = re.compile(
    rf"^\s*(\d+)\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})"
    rf"(?:\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*({FLOAT_RE}))?\s*$"
)
HEADER_MARKERS = ("t_us", "gx", "gy", "gz")

GYRO_AXES = ("gx", "gy", "gz")
DEFAULT_BIAS_S = 3.0
DEFAULT_EXPECTED_FS = 100.0
SYNTHETIC_N = 6000
SYNTHETIC_SIGMA = 0.002
SYNTHETIC_SEED = 0
SYNTHETIC_FS = 100.0
G_MPS2 = 9.80665

BLOCKED_BANNER = (
    "BLOCKED: synthetic still noise, not a real IMU recording.\n"
    "Do not copy this number into firmware until a serial still CSV exists.\n"
    "APP_ZUPT_GYRO_RAD: (blocked)\n"
)


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def _payload_line(raw: str) -> str:
    line = strip_ansi(raw).strip()
    if not line:
        return ""
    if line.startswith("#"):
        return line
    m = ESP_LOG_RE.match(line)
    if m:
        return m.group(1).strip()
    return line


def parse_comment_meta(text: str) -> dict[str, str]:
    meta: dict[str, str] = {}
    for raw in text.splitlines():
        line = _payload_line(raw)
        if not line.startswith("#"):
            continue
        for key, value in COMMENT_KV_RE.findall(line):
            meta[key.lower()] = value.strip().strip(",")
    return meta


def _is_header(line: str) -> bool:
    low = line.lower().replace(" ", "")
    return all(tok in low.split(",") for tok in HEADER_MARKERS)


@dataclass
class StillSeries:
    t_us: object  # np.ndarray
    gx: object
    gy: object
    gz: object
    ax: object | None = None
    ay: object | None = None
    az: object | None = None
    meta: dict[str, str] = field(default_factory=dict)
    source_path: Path | None = None

    @property
    def n(self) -> int:
        return int(len(self.t_us))  # type: ignore[arg-type]


def load_still_text(text: str, *, source_path: Path | None = None) -> StillSeries:
    import numpy as np

    meta = parse_comment_meta(text)
    rows: list[tuple[float, ...]] = []
    seen_header = False
    for raw in text.splitlines():
        line = _payload_line(raw)
        if not line or line.startswith("#"):
            continue
        if not seen_header and _is_header(line):
            seen_header = True
            continue
        m = DATA_RE.match(line)
        if not m:
            continue
        vals = [float(g) for g in m.groups() if g is not None]
        if len(vals) == 4:
            vals.extend((math.nan, math.nan, math.nan))
        rows.append(tuple(vals))

    if len(rows) < 2:
        raise ValueError(
            "need at least 2 still IMU rows (t_us,gx,gy,gz); "
            "check that the file has device timestamps, not PC wall clock"
        )

    arr = np.asarray(rows, dtype=float)
    order = np.argsort(arr[:, 0], kind="mergesort")
    arr = arr[order]
    _, uniq = np.unique(arr[:, 0], return_index=True)
    arr = arr[np.sort(uniq)]
    if arr.shape[0] < 2:
        raise ValueError("t_us collapsed to < 2 unique samples")

    return StillSeries(
        t_us=arr[:, 0].astype(np.int64),
        gx=arr[:, 1],
        gy=arr[:, 2],
        gz=arr[:, 3],
        ax=arr[:, 4],
        ay=arr[:, 5],
        az=arr[:, 6],
        meta=meta,
        source_path=source_path,
    )


def load_still_csv(path: Path) -> StillSeries:
    return load_still_text(path.read_text(encoding="utf-8", errors="replace"), source_path=path)


def median_dt_s(t_us: object) -> float:
    import numpy as np

    t = np.asarray(t_us, dtype=np.int64)
    if t.size < 2:
        raise ValueError("need >= 2 timestamps")
    diffs = np.diff(t)
    diffs = diffs[diffs > 0]
    if diffs.size == 0:
        raise ValueError("t_us is not strictly increasing")
    return float(np.median(diffs)) / 1e6


def is_synthetic(meta: Mapping[str, str], path: Path | None, force: bool) -> bool:
    if force:
        return True
    src = str(meta.get("source", "")).lower()
    if "synthetic" in src:
        return True
    if path is not None and "synthetic" in path.name.lower():
        return True
    return False


def synthesize_still(
    n: int = SYNTHETIC_N,
    fs_hz: float = SYNTHETIC_FS,
    sigma: float = SYNTHETIC_SIGMA,
    seed: int = SYNTHETIC_SEED,
) -> StillSeries:
    import numpy as np

    rng = np.random.default_rng(seed)
    dt_us = int(round(1e6 / fs_hz))
    t_us = np.arange(n, dtype=np.int64) * dt_us
    gyro = rng.normal(0.0, sigma, size=(n, 3))
    accel = np.zeros((n, 3), dtype=float)
    accel[:, 2] = G_MPS2
    meta = {
        "source": "synthetic",
        "sigma": str(sigma),
        "seed": str(seed),
        "accel_unit": "m/s^2",
        "gyro_unit": "rad/s",
        "frame": "hand",
        "bias_subtracted": "0",
    }
    return StillSeries(
        t_us=t_us,
        gx=gyro[:, 0],
        gy=gyro[:, 1],
        gz=gyro[:, 2],
        ax=accel[:, 0],
        ay=accel[:, 1],
        az=accel[:, 2],
        meta=meta,
        source_path=None,
    )


def still_to_csv_text(series: StillSeries) -> str:
    lines = [
        "# source={source} sigma={sigma} seed={seed}".format(
            source=series.meta.get("source", "unknown"),
            sigma=series.meta.get("sigma", ""),
            seed=series.meta.get("seed", ""),
        ),
        f"# accel_unit={series.meta.get('accel_unit', 'unknown')}",
        f"# gyro_unit={series.meta.get('gyro_unit', 'rad/s')}",
        f"# frame={series.meta.get('frame', 'hand')}",
        f"# bias_subtracted={series.meta.get('bias_subtracted', 'unknown')}",
    ]
    if is_synthetic(series.meta, series.source_path, False):
        lines.append("# note=synthetic still noise, do not copy Allan output into firmware")
    lines.append("t_us,gx,gy,gz,ax,ay,az")
    ax = series.ax
    ay = series.ay
    az = series.az
    for i in range(series.n):
        a0 = float(ax[i]) if ax is not None else float("nan")  # type: ignore[index]
        a1 = float(ay[i]) if ay is not None else float("nan")  # type: ignore[index]
        a2 = float(az[i]) if az is not None else float("nan")  # type: ignore[index]
        lines.append(
            f"{int(series.t_us[i])},{float(series.gx[i]):.9g},{float(series.gy[i]):.9g},"
            f"{float(series.gz[i]):.9g},{a0:.9g},{a1:.9g},{a2:.9g}"
        )
    return "\n".join(lines) + "\n"


def _sigfig_ceil(x: float, n: int = 2) -> float:
    if not math.isfinite(x) or x <= 0.0:
        return x
    exp = math.floor(math.log10(x))
    factor = 10 ** (exp - n + 1)
    return math.ceil(x / factor - 1e-15) * factor


def _fmt(x: float, digits: int = 6) -> str:
    if not math.isfinite(x):
        return "nan"
    ax = abs(x)
    if ax != 0.0 and (ax >= 1e4 or ax < 1e-4):
        return f"{x:.{digits}e}"
    return f"{x:.{digits}g}"


@dataclass
class AllanResult:
    series: StillSeries
    dt_s: float
    fs_hz: float
    tau_s: object
    adev: object  # (n_tau, 3)
    avar: object
    params: object  # pandas DataFrame
    sigma: tuple[float, float, float]
    empirical_white: tuple[float, float, float]
    adev_tau_min: tuple[float, float, float]
    bi_tau_s: tuple[float, float, float]
    bi_adev: tuple[float, float, float]
    synthetic: bool
    bias_s: float
    expected_fs: float


def analyze_gyro(
    series: StillSeries,
    *,
    synthetic: bool,
    bias_s: float = DEFAULT_BIAS_S,
    expected_fs: float = DEFAULT_EXPECTED_FS,
) -> AllanResult:
    import numpy as np
    from allan_variance import compute_avar, estimate_parameters

    dt_s = median_dt_s(series.t_us)
    if dt_s <= 0.0:
        raise ValueError("dt_s from t_us must be > 0")
    fs_hz = 1.0 / dt_s
    gyro = np.column_stack(
        [
            np.asarray(series.gx, dtype=float),
            np.asarray(series.gy, dtype=float),
            np.asarray(series.gz, dtype=float),
        ]
    )
    if gyro.shape[0] < 50:
        raise ValueError(f"need more still samples for Allan, got {gyro.shape[0]}")

    tau, avar = compute_avar(gyro, dt=dt_s, input_type="mean")
    adev = np.sqrt(np.maximum(avar, 0.0))
    params, _pred = estimate_parameters(tau, avar, sensor_names=list(GYRO_AXES))

    sigma = tuple(float(np.std(gyro[:, i], ddof=1)) for i in range(3))
    empirical_white = tuple(float(adev[0, i] * math.sqrt(tau[0])) for i in range(3))
    adev_tau_min = tuple(float(adev[0, i]) for i in range(3))
    bi_idx = [int(np.argmin(adev[:, i])) for i in range(3)]
    bi_tau_s = tuple(float(tau[bi_idx[i]]) for i in range(3))
    bi_adev = tuple(float(adev[bi_idx[i], i]) for i in range(3))

    return AllanResult(
        series=series,
        dt_s=dt_s,
        fs_hz=fs_hz,
        tau_s=tau,
        adev=adev,
        avar=avar,
        params=params,
        sigma=sigma,  # type: ignore[arg-type]
        empirical_white=empirical_white,  # type: ignore[arg-type]
        adev_tau_min=adev_tau_min,  # type: ignore[arg-type]
        bi_tau_s=bi_tau_s,  # type: ignore[arg-type]
        bi_adev=bi_adev,  # type: ignore[arg-type]
        synthetic=synthetic,
        bias_s=bias_s,
        expected_fs=expected_fs,
    )


def _param(params: object, axis: str, name: str) -> float:
    try:
        val = params.loc[axis, name]  # type: ignore[union-attr]
        return float(val)
    except Exception:
        return float("nan")


def _bias_comment(result: AllanResult) -> str:
    tau_bi = max(result.bi_tau_s)
    tau_max = float(result.tau_s[-1])  # type: ignore[index]
    at_edge = any(
        abs(result.bi_tau_s[i] - tau_max) / tau_max < 0.05 for i in range(3)
    )
    if at_edge:
        return (
            f"ADEV minimum is at the longest tau ({_fmt(tau_max)} s); "
            "recording is too short to resolve bias instability. "
            "Capture >= 300 s still data before judging APP_GYRO_BIAS_S. "
            f"Current {result.bias_s:g} s is still a reasonable startup mean "
            "if the device is truly still."
        )
    if result.bias_s <= 0.3 * tau_bi:
        return (
            f"APP_GYRO_BIAS_S={result.bias_s:g} s is well below the ADEV-min "
            f"timescale (~{_fmt(tau_bi)} s). Startup mean is in the white-noise "
            "averaging region; 3 s is enough for a still-mean bias."
        )
    if result.bias_s < tau_bi:
        return (
            f"APP_GYRO_BIAS_S={result.bias_s:g} s is approaching the ADEV-min "
            f"timescale (~{_fmt(tau_bi)} s). Mean bias is still useful; "
            "a slightly shorter window reduces flicker wander in the estimate."
        )
    return (
        f"APP_GYRO_BIAS_S={result.bias_s:g} s is at/above the ADEV-min "
        f"timescale (~{_fmt(tau_bi)} s). The window may include bias wander; "
        "consider shortening the startup mean."
    )


def write_outputs(result: AllanResult, out_dir: Path) -> None:
    import numpy as np

    out_dir.mkdir(parents=True, exist_ok=True)
    tau = np.asarray(result.tau_s, dtype=float)
    adev = np.asarray(result.adev, dtype=float)
    avar = np.asarray(result.avar, dtype=float)

    adev_path = out_dir / "allan_adev.csv"
    with adev_path.open("w", encoding="utf-8", newline="\n") as fh:
        fh.write("tau_s,adev_gx,adev_gy,adev_gz,avar_gx,avar_gy,avar_gz\n")
        for i in range(tau.size):
            fh.write(
                f"{tau[i]:.12g},{adev[i, 0]:.12g},{adev[i, 1]:.12g},{adev[i, 2]:.12g},"
                f"{avar[i, 0]:.12g},{avar[i, 1]:.12g},{avar[i, 2]:.12g}\n"
            )

    params_path = out_dir / "allan_params.csv"
    with params_path.open("w", encoding="utf-8", newline="\n") as fh:
        fh.write(
            "axis,quantization,white,flicker,walk,ramp,"
            "still_std_rad_s,empirical_white_rad_s_sqrt,adev_at_tau_min,"
            "bias_instability_tau_s,bias_instability_adev\n"
        )
        fh.write(
            "# units for gyro in rad/s: quantization=rad, white=rad*s^0.5, "
            "flicker=rad/s, walk=rad/s^1.5, ramp=rad/s^2 "
            "(allan-variance estimate_parameters)\n"
        )
        for i, axis in enumerate(GYRO_AXES):
            fh.write(
                f"{axis},"
                f"{_fmt(_param(result.params, axis, 'quantization'))},"
                f"{_fmt(_param(result.params, axis, 'white'))},"
                f"{_fmt(_param(result.params, axis, 'flicker'))},"
                f"{_fmt(_param(result.params, axis, 'walk'))},"
                f"{_fmt(_param(result.params, axis, 'ramp'))},"
                f"{_fmt(result.sigma[i])},"
                f"{_fmt(result.empirical_white[i])},"
                f"{_fmt(result.adev_tau_min[i])},"
                f"{_fmt(result.bi_tau_s[i])},"
                f"{_fmt(result.bi_adev[i])}\n"
            )

    zupt_5sigma = 5.0 * max(result.sigma)
    zupt_5adev = 5.0 * max(result.adev_tau_min)
    zupt_suggest = _sigfig_ceil(max(zupt_5sigma, zupt_5adev), 2)
    src = result.series.meta.get("source", "unknown")
    in_name = result.series.source_path.name if result.series.source_path else "(in-memory)"
    duration_s = float(result.series.n - 1) * result.dt_s
    fs_note = ""
    if result.expected_fs > 0.0:
        rel = abs(result.fs_hz - result.expected_fs) / result.expected_fs
        if rel > 0.05:
            fs_note = (
                f" WARNING: fs={result.fs_hz:.3f} Hz is >5% away from "
                f"APP_SAMPLE_HZ={result.expected_fs:g}"
            )

    lines = [
        "# 3.3 still / Allan suggestion — review before copying into app_config.h",
        f"source: {src}",
        f"input: {in_name}",
        f"samples: {result.series.n}",
        f"duration_s: {_fmt(duration_s, 4)}",
        f"dt_s: {_fmt(result.dt_s, 8)}  (median diff of t_us, not PC wall clock)",
        f"fs_hz: {_fmt(result.fs_hz, 4)}{fs_note}",
        f"accel_unit: {result.series.meta.get('accel_unit', 'unspecified')}",
        f"gyro_unit: {result.series.meta.get('gyro_unit', 'rad/s')}",
        f"frame: {result.series.meta.get('frame', 'hand')}",
        f"bias_subtracted: {result.series.meta.get('bias_subtracted', 'unknown')}",
        "",
        "# still-rate std (rad/s)",
        f"sigma_gx_rad_s: {_fmt(result.sigma[0])}",
        f"sigma_gy_rad_s: {_fmt(result.sigma[1])}",
        f"sigma_gz_rad_s: {_fmt(result.sigma[2])}",
        "",
        "# approximate white noise N: ADEV(tau_min)*sqrt(tau_min)  [rad * s^-0.5]",
        f"white_emp_gx: {_fmt(result.empirical_white[0])}",
        f"white_emp_gy: {_fmt(result.empirical_white[1])}",
        f"white_emp_gz: {_fmt(result.empirical_white[2])}",
        f"white_fit_gx: {_fmt(_param(result.params, 'gx', 'white'))}",
        f"white_fit_gy: {_fmt(_param(result.params, 'gy', 'white'))}",
        f"white_fit_gz: {_fmt(_param(result.params, 'gz', 'white'))}",
        "",
        "# bias instability: library flicker (rad/s) and empirical min(ADEV)",
        f"flicker_fit_gx_rad_s: {_fmt(_param(result.params, 'gx', 'flicker'))}",
        f"flicker_fit_gy_rad_s: {_fmt(_param(result.params, 'gy', 'flicker'))}",
        f"flicker_fit_gz_rad_s: {_fmt(_param(result.params, 'gz', 'flicker'))}",
        f"bias_instability_gx_rad_s: {_fmt(result.bi_adev[0])}",
        f"bias_instability_gy_rad_s: {_fmt(result.bi_adev[1])}",
        f"bias_instability_gz_rad_s: {_fmt(result.bi_adev[2])}",
        f"bias_instability_tau_gx_s: {_fmt(result.bi_tau_s[0])}",
        f"bias_instability_tau_gy_s: {_fmt(result.bi_tau_s[1])}",
        f"bias_instability_tau_gz_s: {_fmt(result.bi_tau_s[2])}",
        f"adev_at_tau_min_max_rad_s: {_fmt(max(result.adev_tau_min))}",
        "",
        "# vs firmware 3.1 startup still-mean",
        f"APP_GYRO_BIAS_S_current: {result.bias_s:g}",
        f"APP_GYRO_BIAS_S_comment: {_bias_comment(result)}",
        "",
        "# future ZUPT (not implemented on ESP32). 5*still-std vs 5*ADEV(tau_min).",
        "# This is tighter than APP_GYRO_BIAS_MAX_STILL_RAD (3.1 motion reject).",
        f"zupt_5sigma_max_axis_rad_s: {_fmt(zupt_5sigma)}",
        f"zupt_5_adev_tau_min_rad_s: {_fmt(zupt_5adev)}",
    ]
    if result.synthetic:
        lines.extend(["", BLOCKED_BANNER.rstrip("\n")])
    else:
        lines.extend(
            [
                "",
                f"APP_ZUPT_GYRO_RAD: {zupt_suggest:g}  # suggested, human review required",
            ]
        )

    (out_dir / "zupt_suggestion.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Allan variance of still IMU CSV (PC offline, task 3.3)"
    )
    p.add_argument(
        "--input",
        type=Path,
        default=None,
        help="still CSV or UART log with t_us,gx,gy,gz[,ax,ay,az]",
    )
    p.add_argument(
        "--out",
        type=Path,
        default=Path("analysis/out"),
        help="output directory (allan_adev.csv, allan_params.csv, zupt_suggestion.txt)",
    )
    p.add_argument(
        "--synthesize",
        action="store_true",
        help="generate synthetic still noise (always BLOCKED, do not write firmware)",
    )
    p.add_argument(
        "--synthetic",
        action="store_true",
        help="force BLOCKED marking even if the file looks like a device recording",
    )
    p.add_argument(
        "--bias-s",
        type=float,
        default=DEFAULT_BIAS_S,
        help="current APP_GYRO_BIAS_S to compare against ADEV-min tau (default 3.0)",
    )
    p.add_argument(
        "--expected-fs",
        type=float,
        default=DEFAULT_EXPECTED_FS,
        help="APP_SAMPLE_HZ to compare with 1/median(dt) (default 100)",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.input is None and not args.synthesize:
        print("error: provide --input CSV/log or --synthesize", file=sys.stderr)
        return 2

    if args.synthesize:
        series = synthesize_still()
        args.out.mkdir(parents=True, exist_ok=True)
        synth_path = args.out / "synthetic_still.csv"
        synth_path.write_text(still_to_csv_text(series), encoding="utf-8")
        series.source_path = synth_path
        synthetic = True
    else:
        path = args.input.expanduser().resolve()
        if not path.is_file():
            print(f"error: input not found: {path}", file=sys.stderr)
            return 2
        series = load_still_csv(path)
        synthetic = is_synthetic(series.meta, path, args.synthetic)

    result = analyze_gyro(
        series,
        synthetic=synthetic,
        bias_s=args.bias_s,
        expected_fs=args.expected_fs,
    )
    write_outputs(result, args.out)
    print(
        f"wrote {args.out / 'allan_adev.csv'}, "
        f"{args.out / 'allan_params.csv'}, "
        f"{args.out / 'zupt_suggestion.txt'}"
        + ("  [BLOCKED synthetic]" if synthetic else "")
    )
    print(
        f"dt_s={result.dt_s:.6g} fs_hz={result.fs_hz:.4g} n={result.series.n} "
        f"sigma=[{', '.join(_fmt(s) for s in result.sigma)}] rad/s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
