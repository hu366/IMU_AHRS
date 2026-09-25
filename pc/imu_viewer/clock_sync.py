"""ESP32 esp_timer to PC monotonic_ns clock mapping."""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass


@dataclass(frozen=True)
class SyncExchange:
    sync_id: int | None
    boot_id: int
    t1_pc_ns: int
    t2_esp_us: int
    t3_esp_us: int
    t4_pc_ns: int
    rtt_ns: int
    esp_mid_us: float
    pc_mid_ns: float


@dataclass(frozen=True)
class ClockMapQuality:
    ready: bool
    boot_id: int | None
    sample_count: int
    used_sample_count: int
    rtt_min_ns: int | None
    rtt_median_ns: float | None
    residual_rms_ns: float | None
    residual_median_abs_ns: float | None
    slope_ns_per_esp_us: float | None
    offset_ns: float | None
    drift_ppm: float | None
    map_version: int
    invalid_count: int = 0
    status: str = "not_ready"

    @property
    def a(self) -> float | None:
        return self.slope_ns_per_esp_us

    @property
    def b(self) -> float | None:
        return self.offset_ns

    @property
    def residual_ns(self) -> float | None:
        return self.residual_rms_ns

    @property
    def n_samples(self) -> int:
        return self.sample_count

    @property
    def residual_std_ns(self) -> float | None:
        return self.residual_rms_ns


class AffineClockMapper:
    """Estimate t_pc_ns = a * t_esp_us + b for one ESP boot session."""

    def __init__(
        self,
        min_samples: int = 8,
        *,
        min_points: int | None = None,
        max_samples: int = 128,
        keep_fraction: float = 0.5,
        huber_delta: float = 1.345,
    ) -> None:
        if min_points is not None:
            min_samples = min_points
        if min_samples < 2:
            raise ValueError("min_samples must be at least 2")
        if max_samples < min_samples:
            raise ValueError("max_samples must be >= min_samples")
        if not 0.0 < keep_fraction <= 1.0:
            raise ValueError("keep_fraction must be in (0, 1]")
        self.min_samples = int(min_samples)
        self.max_samples = int(max_samples)
        self.keep_fraction = float(keep_fraction)
        self.huber_delta = float(huber_delta)
        self._boot_id: int | None = None
        self._exchanges: list[SyncExchange] = []
        self._a: float | None = None
        self._b: float | None = None
        self._version = 0
        self._used_count = 0
        self._rtt_min: int | None = None
        self._rtt_median: float | None = None
        self._rms: float | None = None
        self._mad: float | None = None
        self._invalid_count = 0

    @property
    def boot_id(self) -> int | None:
        return self._boot_id

    @property
    def map_version(self) -> int:
        return self._version

    @property
    def exchanges(self) -> tuple[SyncExchange, ...]:
        return tuple(self._exchanges)

    def reset(self, boot_id: int | None = None) -> None:
        self._boot_id = boot_id
        self._exchanges.clear()
        self._a = None
        self._b = None
        self._used_count = 0
        self._rtt_min = None
        self._rtt_median = None
        self._rms = None
        self._mad = None
        self._version += 1

    def add_exchange(
        self,
        *,
        t1_pc_ns: int,
        t2_esp_us: int,
        t3_esp_us: int,
        t4_pc_ns: int,
        boot_id: int,
        sync_id: int | None = None,
    ) -> None:
        """Add one matched TSQ/TSR exchange."""
        try:
            t1, t2, t3, t4, boot = (
                int(t1_pc_ns),
                int(t2_esp_us),
                int(t3_esp_us),
                int(t4_pc_ns),
                int(boot_id),
            )
        except (TypeError, ValueError, OverflowError):
            self._invalid_count += 1
            return
        if min(t1, t2, t3, t4) < 0 or t4 < t1 or t3 < t2:
            self._invalid_count += 1
            return
        rtt = (t4 - t1) - (t3 - t2) * 1000
        if rtt < 0:
            self._invalid_count += 1
            return
        if self._boot_id != boot:
            self.reset(boot)
        self._exchanges.append(
            SyncExchange(
                sync_id=sync_id,
                boot_id=boot,
                t1_pc_ns=t1,
                t2_esp_us=t2,
                t3_esp_us=t3,
                t4_pc_ns=t4,
                rtt_ns=int(rtt),
                esp_mid_us=(t2 + t3) / 2.0,
                pc_mid_ns=(t1 + t4) / 2.0,
            )
        )
        if len(self._exchanges) > self.max_samples:
            del self._exchanges[: len(self._exchanges) - self.max_samples]
        self._fit()

    def add_response(
        self,
        *,
        t1_pc_ns: int,
        t4_pc_ns: int,
        boot_id: int,
        t2_esp_us: int,
        t3_esp_us: int,
        sync_id: int | None = None,
    ) -> None:
        self.add_exchange(
            t1_pc_ns=t1_pc_ns,
            t2_esp_us=t2_esp_us,
            t3_esp_us=t3_esp_us,
            t4_pc_ns=t4_pc_ns,
            boot_id=boot_id,
            sync_id=sync_id,
        )

    def map_esp_us(self, t_esp_us: int, boot_id: int) -> int | None:
        if (
            self._a is None
            or self._b is None
            or self._boot_id is None
            or int(boot_id) != self._boot_id
            or len(self._exchanges) < self.min_samples
        ):
            return None
        value = self._a * float(t_esp_us) + self._b
        return int(round(value)) if math.isfinite(value) else None

    def quality(self) -> ClockMapQuality:
        ready = self._a is not None and self._b is not None and len(self._exchanges) >= self.min_samples
        status = "ready" if ready else ("collecting" if self._exchanges else "not_ready")
        return ClockMapQuality(
            ready=ready,
            boot_id=self._boot_id,
            sample_count=len(self._exchanges),
            used_sample_count=self._used_count,
            rtt_min_ns=self._rtt_min,
            rtt_median_ns=self._rtt_median,
            residual_rms_ns=self._rms,
            residual_median_abs_ns=self._mad,
            slope_ns_per_esp_us=self._a,
            offset_ns=self._b,
            drift_ppm=None if self._a is None else (self._a / 1000.0 - 1.0) * 1e6,
            map_version=self._version,
            invalid_count=self._invalid_count,
            status=status,
        )

    def _fit(self) -> None:
        if len(self._exchanges) < 2:
            return
        ordered = sorted(self._exchanges, key=lambda item: item.rtt_ns)
        keep_n = max(2, int(math.ceil(len(ordered) * self.keep_fraction)))
        if len(ordered) >= self.min_samples:
            keep_n = max(self.min_samples, keep_n)
        selected = ordered[:keep_n]
        a, b = _robust_regression(
            [item.esp_mid_us for item in selected],
            [item.pc_mid_ns for item in selected],
            self.huber_delta,
        )
        if a is None or b is None:
            return
        self._a, self._b = a, b
        self._used_count = len(selected)
        self._rtt_min = min(item.rtt_ns for item in self._exchanges)
        self._rtt_median = float(statistics.median(item.rtt_ns for item in self._exchanges))
        residuals = [item.pc_mid_ns - (a * item.esp_mid_us + b) for item in self._exchanges]
        self._rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
        self._mad = float(statistics.median(abs(value) for value in residuals))
        self._version += 1


ClockSynchronizer = AffineClockMapper


def _robust_regression(
    xs: list[float], ys: list[float], huber_delta: float
) -> tuple[float | None, float | None]:
    if len(xs) < 2:
        return None, None
    x0 = sum(xs) / len(xs)
    y0 = sum(ys) / len(ys)
    centered_x = [x - x0 for x in xs]
    centered_y = [y - y0 for y in ys]
    weights = [1.0] * len(xs)
    slope = 1000.0
    intercept_centered = 0.0
    for _ in range(8):
        sw = sum(weights)
        mx = sum(weight * x for weight, x in zip(weights, centered_x)) / sw
        my = sum(weight * y for weight, y in zip(weights, centered_y)) / sw
        denom = sum(weight * (x - mx) ** 2 for weight, x in zip(weights, centered_x))
        if denom <= 0.0:
            return None, None
        slope = sum(
            weight * (x - mx) * (y - my)
            for weight, x, y in zip(weights, centered_x, centered_y)
        ) / denom
        intercept_centered = my - slope * mx
        residuals = [
            y - (slope * x + intercept_centered)
            for x, y in zip(centered_x, centered_y)
        ]
        scale = max(statistics.median(abs(value) for value in residuals) * 1.4826, 1.0)
        cutoff = max(1.0, huber_delta * scale)
        weights = [
            1.0 if abs(residual) <= cutoff else cutoff / abs(residual)
            for residual in residuals
        ]
    return slope, y0 + intercept_centered - slope * x0
