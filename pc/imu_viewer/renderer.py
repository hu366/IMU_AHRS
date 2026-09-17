"""60 Hz VPython box. Reads PoseStore copies; never talks to BLE."""

from __future__ import annotations

import logging
import threading

from imu_viewer.orientation import quat_to_axis_up
from imu_viewer.pose_state import PoseStore, format_status

log = logging.getLogger("imu_viewer.renderer")

RENDER_HZ = 60
# Identity pose: slab in XY, thin in Z. Long edge along Y, short along X.
_BOX_SIZE_X = 1.2
_BOX_SIZE_Y = 2.4
_BOX_SIZE_Z = 0.22
_AXIS_EXTRA = 0.45


def run_renderer(
    store: PoseStore,
    stop_event: threading.Event,
    *,
    hz: int = RENDER_HZ,
) -> None:
    """Block on the calling thread until the window closes or stop_event is set.

    VPython is imported here so protocol/BLE tests can run without a display.
    """
    try:
        from vpython import arrow, box, canvas, color, label, rate, vector
    except ImportError as exc:
        raise SystemExit(
            "vpython is required for the viewer. "
            "Install with: pip install -r requirements.txt"
        ) from exc

    # Camera only: look along -Z so +Z points at the viewer.
    # Screen: X up, Y left, Z toward you. Does not remap quaternions (SH-02).
    scene = canvas(
        title="IMU-AHRS viewer",
        width=960,
        height=720,
        background=color.gray(0.12),
        up=vector(1, 0, 0),
        forward=vector(0, 0, -1),
        center=vector(0, 0, 0),
        range=3.2,
        caption=(
            "View: Z toward you, Y left, X up. "
            "Hand frame: X = fingers (red), Y = right-hand (green), "
            "Z = out of palm (blue). Close the window or press q / Ctrl+C to quit.\n"
        ),
    )

    _world_arrow(arrow, label, vector, color.red, (1.8, 0, 0), "Xw")
    _world_arrow(arrow, label, vector, color.green, (0, 1.8, 0), "Yw")
    _world_arrow(arrow, label, vector, color.blue, (0, 0, 1.8), "Zw")

    body = box(
        pos=vector(0, 0, 0),
        axis=vector(_BOX_SIZE_X, 0, 0),
        up=vector(0, 1, 0),
        length=_BOX_SIZE_X,
        height=_BOX_SIZE_Y,
        width=_BOX_SIZE_Z,
        color=color.orange,
        opacity=0.92,
    )
    body_x = arrow(
        pos=vector(0, 0, 0),
        axis=vector(_BOX_SIZE_X / 2 + _AXIS_EXTRA, 0, 0),
        shaftwidth=0.05,
        color=color.red,
    )
    body_y = arrow(
        pos=vector(0, 0, 0),
        axis=vector(0, _BOX_SIZE_Y / 2 + _AXIS_EXTRA, 0),
        shaftwidth=0.05,
        color=color.green,
    )
    body_z = arrow(
        pos=vector(0, 0, 0),
        axis=vector(0, 0, _BOX_SIZE_Z / 2 + _AXIS_EXTRA),
        shaftwidth=0.05,
        color=color.blue,
    )

    status = label(
        pos=vector(2.2, 0, 0),
        text="starting...",
        box=False,
        height=16,
        color=color.white,
        align="center",
    )

    log.info("renderer loop at %s Hz", hz)
    try:
        while not stop_event.is_set():
            rate(hz)
            if _quit_requested(scene):
                break
            snap = store.snapshot()
            axis, up = quat_to_axis_up(snap.pose.quaternion)
            ax = vector(*axis)
            upv = vector(*up)
            body.axis = ax * _BOX_SIZE_X
            body.up = upv
            body_x.axis = ax * (_BOX_SIZE_X / 2 + _AXIS_EXTRA)
            body_y.axis = upv * (_BOX_SIZE_Y / 2 + _AXIS_EXTRA)
            body_z.axis = ax.cross(upv) * (_BOX_SIZE_Z / 2 + _AXIS_EXTRA)
            status.text = format_status(snap)
    except KeyboardInterrupt:
        log.info("renderer interrupted")
    finally:
        stop_event.set()
        try:
            scene.delete()
        except Exception:
            pass
        log.info("renderer stopped")


def _world_arrow(arrow, label, vector, col, axis, name: str) -> None:
    axis_v = vector(*axis)
    arrow(pos=vector(0, 0, 0), axis=axis_v, shaftwidth=0.03, color=col, opacity=0.45)
    label(pos=axis_v, text=name, box=False, height=12, color=col)


def _quit_requested(scene) -> bool:
    try:
        from vpython import keysdown
    except ImportError:
        keysdown = None
    if keysdown is not None:
        try:
            keys = {k.lower() for k in keysdown()}
        except Exception:
            keys = set()
        if "q" in keys or "escape" in keys or "esc" in keys:
            return True
    visible = getattr(scene, "visible", True)
    if visible is False:
        return True
    if getattr(scene, "_destroy", False) or getattr(scene, "_destroyed", False):
        return True
    return False
