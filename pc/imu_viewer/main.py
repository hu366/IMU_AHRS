"""Assemble BLE (or demo feeder), pose store, and the 60 Hz renderer."""

from __future__ import annotations

import argparse
import logging
import math
import sys
import threading
import time

from imu_viewer.orientation import Quaternion
from imu_viewer.pose_state import PoseStore
from imu_viewer.renderer import RENDER_HZ, run_renderer

log = logging.getLogger("imu_viewer")

_DEMO_PERIOD_S = 8.0
_DEMO_FEED_HZ = 100.0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="imu_viewer",
        description="IMU-AHRS PC viewer: Q,w,x,y,z over BLE -> 60 Hz VPython box.",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="offline demo: feed slow X/Y/Z rotations, no BLE",
    )
    parser.add_argument(
        "--name",
        default="IMU-AHRS",
        help="BLE advertised name (default: IMU-AHRS)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="seconds without a frame before the pose is marked stale (0.5–1.0)",
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=10.0,
        help="BLE scan timeout in seconds before retrying",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="do not print each parsed quaternion in BLE mode",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    if args.timeout <= 0:
        log.error("--timeout must be positive")
        return 2

    store = PoseStore(timeout_s=args.timeout)
    stop_event = threading.Event()
    worker: threading.Thread
    if args.demo:
        store.set_mode("demo")
        store.set_connected(True)
        worker = threading.Thread(
            target=_demo_loop,
            args=(store, stop_event),
            name="demo-feeder",
            daemon=True,
        )
        log.info("demo mode: cycling 90°+ rotations about X, Y, Z (no BLE)")
    else:
        store.set_mode("ble")
        from imu_viewer.ble_client import BleClient

        ble = BleClient(
            store,
            name=args.name,
            scan_timeout=args.scan_timeout,
            print_frames=not args.quiet,
        )
        worker = threading.Thread(
            target=ble.run,
            args=(stop_event,),
            name="bleak",
            daemon=True,
        )
        log.info("BLE mode: scanning for %s", args.name)

    worker.start()
    try:
        run_renderer(store, stop_event, hz=RENDER_HZ)
    except KeyboardInterrupt:
        log.info("Ctrl+C")
    finally:
        stop_event.set()
        worker.join(timeout=2.5)
        if worker.is_alive():
            log.warning("background thread still running; process exit will drop it")
    return 0


def demo_quaternion(t: float) -> Quaternion:
    """Slow full-turn about X, then Y, then Z, 8 s per axis."""
    period = _DEMO_PERIOD_S
    axis_index = int(t / period) % 3
    half = 0.5 * ((t % period) / period) * 2.0 * math.pi
    c, s = math.cos(half), math.sin(half)
    if axis_index == 0:
        return (c, s, 0.0, 0.0)
    if axis_index == 1:
        return (c, 0.0, s, 0.0)
    return (c, 0.0, 0.0, s)


def _demo_loop(store: PoseStore, stop_event: threading.Event) -> None:
    t0 = time.monotonic()
    dt = 1.0 / _DEMO_FEED_HZ
    while not stop_event.is_set():
        q = demo_quaternion(time.monotonic() - t0)
        store.ingest((q,))
        stop_event.wait(dt)


if __name__ == "__main__":
    sys.exit(main())
