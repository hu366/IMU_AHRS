"""BLE central for IMU Q/QT frames and TSQ/TSR clock synchronization."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import threading
import time
from collections.abc import Callable

from imu_viewer.alignment import CameraFrameAligner
from imu_viewer.clock_sync import AffineClockMapper, ClockMapQuality
from imu_viewer.pose_state import PoseStore
from imu_viewer.protocol import (
    ProtocolDecoder,
    ProtocolEvent,
    Quaternion,
    TimeSyncResponse,
    TimedQuaternionFrame,
)

log = logging.getLogger("imu_viewer.ble")

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEFAULT_NAME = "IMU-AHRS"

NotifyPayload = bytes | bytearray | memoryview | str
MonotonicNsClock = Callable[[], int]


class NotifyHandler:
    """Shared path for real bleak notifications and fake test callbacks."""

    def __init__(
        self,
        store: PoseStore,
        *,
        print_frames: bool = False,
        on_frame: Callable[[Quaternion], None] | None = None,
        on_timed_frame: Callable[[TimedQuaternionFrame], None] | None = None,
        on_sync_response: Callable[[TimeSyncResponse, int], None] | None = None,
        on_event: Callable[[ProtocolEvent, int], None] | None = None,
        monotonic_ns: MonotonicNsClock = time.monotonic_ns,
    ) -> None:
        self.decoder = ProtocolDecoder()
        self.store = store
        self.print_frames = print_frames
        self.on_frame = on_frame
        self.on_timed_frame = on_timed_frame
        self.on_sync_response = on_sync_response
        self.on_event = on_event
        self._monotonic_ns = monotonic_ns

    def on_notify(self, _sender: object, data: NotifyPayload) -> None:
        """Capture t4 immediately, then decode and dispatch complete lines."""
        t4_pc_ns = self._monotonic_ns()
        events = self.decoder.feed_events(_as_bytes(data))
        quaternions: list[Quaternion] = []
        latest_sequence: int | None = None
        for event in events:
            if isinstance(event, TimedQuaternionFrame):
                q = event.quaternion_wxyz
                quaternions.append(q)
                latest_sequence = event.sequence
                if self.print_frames:
                    print(
                        "QT,"
                        f"{event.boot_id},{event.sequence},{event.t_esp_us},"
                        f"{q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f}",
                        flush=True,
                    )
                if self.on_timed_frame is not None:
                    self.on_timed_frame(event)
                if self.on_frame is not None:
                    self.on_frame(q)
            elif isinstance(event, TimeSyncResponse):
                if self.on_sync_response is not None:
                    self.on_sync_response(event, t4_pc_ns)
            else:
                quaternions.append(event)
                if self.print_frames:
                    w, x, y, z = event
                    print(f"Q,{w:.4f},{x:.4f},{y:.4f},{z:.4f}", flush=True)
                if self.on_frame is not None:
                    self.on_frame(event)
            if self.on_event is not None:
                self.on_event(event, t4_pc_ns)
        self.store.ingest(
            quaternions,
            decoder=self.decoder,
            sequence=latest_sequence,
        )


class BleClient:
    """Scan, reconnect, receive QT, and continuously synchronize ESP time."""

    SERVICE_UUID = NUS_SERVICE_UUID
    RX_UUID = NUS_RX_UUID
    TX_UUID = NUS_TX_UUID
    DEFAULT_NAME = DEFAULT_NAME

    def __init__(
        self,
        store: PoseStore,
        *,
        name: str = DEFAULT_NAME,
        scan_timeout: float = 10.0,
        retry_s: float = 2.0,
        print_frames: bool = True,
        clock_mapper: AffineClockMapper | None = None,
        frame_aligner: CameraFrameAligner | None = None,
        sync_period_s: float = 1.0,
        sync_report: bool = False,
        monotonic_ns: MonotonicNsClock = time.monotonic_ns,
    ) -> None:
        if sync_period_s <= 0.0:
            raise ValueError("sync_period_s must be positive")
        self._store = store
        self._name = name
        self._scan_timeout = scan_timeout
        self._retry_s = retry_s
        self._sync_period_s = sync_period_s
        self._sync_report = bool(sync_report)
        self._monotonic_ns = monotonic_ns
        self._clock_mapper = clock_mapper or AffineClockMapper()
        self._frame_aligner = frame_aligner
        self._pending_sync: dict[int, int] = {}
        self._sync_write_duration_ns: dict[int, int] = {}
        self._next_sync_id = 0
        self._observed_boot_id: int | None = None
        self._handler = NotifyHandler(
            store,
            print_frames=print_frames,
            on_timed_frame=self._on_timed_frame,
            on_sync_response=self._on_sync_response,
            monotonic_ns=monotonic_ns,
        )
        self._client = None
        self._loop: asyncio.AbstractEventLoop | None = None

    @property
    def decoder(self) -> ProtocolDecoder:
        return self._handler.decoder

    @property
    def clock_mapper(self) -> AffineClockMapper:
        return self._clock_mapper

    @property
    def frame_aligner(self) -> CameraFrameAligner | None:
        return self._frame_aligner

    def handle_notify(self, sender: object, data: NotifyPayload) -> None:
        """Bleak notification callback. It never touches the renderer."""
        self._handler.on_notify(sender, data)

    def run(self, stop_event: threading.Event) -> None:
        asyncio.run(self._run(stop_event))

    async def _run(self, stop_event: threading.Event) -> None:
        self._loop = asyncio.get_running_loop()
        try:
            while not stop_event.is_set():
                try:
                    device = await self._scan()
                    if device is None:
                        self._store.set_connected(False)
                        log.warning(
                            "device %r / NUS %s not found (scan %.0fs). Retrying...",
                            self._name,
                            self.SERVICE_UUID,
                            self._scan_timeout,
                        )
                        await _sleep_until(stop_event, self._retry_s)
                        continue
                    await self._connect_and_listen(device, stop_event)
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    self._store.set_connected(False)
                    _log_ble_error(exc)
                    await _sleep_until(stop_event, self._retry_s)
        finally:
            await self._disconnect()
            self._loop = None

    async def _scan(self):
        from bleak import BleakScanner
        from bleak.backends.device import BLEDevice
        from bleak.backends.scanner import AdvertisementData

        name = self._name
        service = self.SERVICE_UUID.lower()

        def match(device: BLEDevice, adv: AdvertisementData) -> bool:
            if name and device.name == name:
                return True
            return service in [uuid.lower() for uuid in (adv.service_uuids or [])]

        log.info("scanning for %r or service %s", name, self.SERVICE_UUID)
        return await BleakScanner.find_device_by_filter(
            match,
            timeout=self._scan_timeout,
        )

    async def _connect_and_listen(self, device, stop_event: threading.Event) -> None:
        from bleak import BleakClient

        disconnected = asyncio.Event()

        def on_disconnect(_client) -> None:
            self._store.set_connected(False)
            disconnected.set()
            log.info("disconnected from %s (%s)", device.name, device.address)

        log.info("connecting to %s (%s)", device.name, device.address)
        async with BleakClient(device, disconnected_callback=on_disconnect) as client:
            self._client = client
            self._reset_connection_sync_state()
            self._store.set_connected(True)
            try:
                await client.start_notify(self.TX_UUID, self.handle_notify)
            except Exception as exc:
                _log_ble_error(exc)
                raise
            log.info("subscribed to TX %s; starting %.2fs clock sync", self.TX_UUID, self._sync_period_s)
            sync_task = asyncio.create_task(
                self._sync_loop(client, stop_event, disconnected),
                name="imu-time-sync",
            )
            try:
                while not stop_event.is_set() and not disconnected.is_set():
                    if not client.is_connected:
                        break
                    await asyncio.sleep(0.1)
            finally:
                sync_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await sync_task
                self._pending_sync.clear()
                self._sync_write_duration_ns.clear()
        self._client = None
        self._store.set_connected(False)

    async def _sync_loop(self, client, stop_event: threading.Event, disconnected: asyncio.Event) -> None:
        while not stop_event.is_set() and not disconnected.is_set() and client.is_connected:
            await self._send_sync_request(client)
            await _sleep_until(stop_event, self._sync_period_s, disconnected)

    async def _send_sync_request(self, client) -> None:
        self._next_sync_id = (self._next_sync_id + 1) & 0xFFFFFFFF
        sync_id = self._next_sync_id
        t1_pc_ns = self._monotonic_ns()
        self._pending_sync[sync_id] = t1_pc_ns
        self._prune_pending_sync(t1_pc_ns)
        try:
            await client.write_gatt_char(
                self.RX_UUID,
                f"TSQ,{sync_id}\n".encode("ascii"),
                response=False,
            )
        except Exception:
            self._pending_sync.pop(sync_id, None)
            self._sync_write_duration_ns.pop(sync_id, None)
            raise
        if sync_id in self._pending_sync:
            self._sync_write_duration_ns[sync_id] = self._monotonic_ns() - t1_pc_ns

    def _on_timed_frame(self, frame: TimedQuaternionFrame) -> None:
        if self._observed_boot_id != frame.boot_id:
            self._observed_boot_id = frame.boot_id
            self._pending_sync.clear()
            self._sync_write_duration_ns.clear()
            self._clock_mapper.reset(frame.boot_id)
            if self._frame_aligner is not None:
                self._frame_aligner.reset(frame.boot_id)
            log.info("observed ESP boot_id=%d; invalidated prior clock map", frame.boot_id)
        if self._frame_aligner is not None:
            self._frame_aligner.ingest_timed_quaternion(frame, self._clock_mapper)

    def _on_sync_response(self, response: TimeSyncResponse, t4_pc_ns: int) -> None:
        if self._observed_boot_id is not None and response.boot_id != self._observed_boot_id:
            log.warning(
                "ignore stale TSR boot_id=%d while receiving boot_id=%d",
                response.boot_id,
                self._observed_boot_id,
            )
            return
        t1_pc_ns = self._pending_sync.pop(response.sync_id, None)
        if t1_pc_ns is None:
            log.debug("ignore unmatched TSR sync_id=%d", response.sync_id)
            return
        write_duration_ns = self._sync_write_duration_ns.pop(response.sync_id, None)
        if self._observed_boot_id != response.boot_id:
            self._observed_boot_id = response.boot_id
            self._pending_sync.clear()
            self._sync_write_duration_ns.clear()
            if self._frame_aligner is not None:
                self._frame_aligner.reset(response.boot_id)
        self._clock_mapper.add_exchange(
            t1_pc_ns=t1_pc_ns,
            t2_esp_us=response.t2_receive_us,
            t3_esp_us=response.t3_send_us,
            t4_pc_ns=t4_pc_ns,
            boot_id=response.boot_id,
            sync_id=response.sync_id,
        )
        if self._frame_aligner is not None:
            self._frame_aligner.refresh_clock_map(self._clock_mapper)
        quality = self._clock_mapper.quality()
        if self._sync_report:
            exchanges = self._clock_mapper.exchanges
            latest_rtt_ns = exchanges[-1].rtt_ns if exchanges else None
            _log_sync_report(response, quality, latest_rtt_ns, write_duration_ns)
        log.debug(
            "TSR id=%d rtt=%s samples=%d ready=%s",
            response.sync_id,
            quality.rtt_median_ns,
            quality.sample_count,
            quality.ready,
        )

    def _reset_connection_sync_state(self) -> None:
        self._pending_sync.clear()
        self._sync_write_duration_ns.clear()
        self._observed_boot_id = None
        self._clock_mapper.reset()
        if self._frame_aligner is not None:
            self._frame_aligner.reset()

    def _prune_pending_sync(self, now_ns: int) -> None:
        cutoff = now_ns - 10_000_000_000
        for sync_id, t1_pc_ns in tuple(self._pending_sync.items()):
            if t1_pc_ns < cutoff:
                del self._pending_sync[sync_id]
                self._sync_write_duration_ns.pop(sync_id, None)

    async def _disconnect(self) -> None:
        client = self._client
        self._client = None
        if client is not None:
            try:
                if getattr(client, "is_connected", False):
                    await client.disconnect()
            except Exception:
                log.debug("disconnect failed", exc_info=True)
        self._pending_sync.clear()
        self._sync_write_duration_ns.clear()
        self._store.set_connected(False)
        log.info("BLE client stopped")


def _as_bytes(data: NotifyPayload) -> bytes:
    return data.encode("utf-8") if isinstance(data, str) else bytes(data)


def _log_sync_report(
    response: TimeSyncResponse,
    quality: ClockMapQuality,
    latest_rtt_ns: int | None,
    write_duration_ns: int | None,
) -> None:
    """Emit the small set of values needed for a real-device sync check."""
    def ms(value: int | float | None) -> str:
        return "-" if value is None else f"{float(value) / 1_000_000.0:.3f}"

    slope = "-" if quality.slope_ns_per_esp_us is None else f"{quality.slope_ns_per_esp_us:.6f}"
    drift = "-" if quality.drift_ppm is None else f"{quality.drift_ppm:.2f}"
    log.info(
        "SYNC boot_id=%d id=%d total_samples=%d used_samples=%d ready=%s "
        "write_call_ms=%s esp_service_ms=%s rtt_ms=%s rtt_min_ms=%s "
        "rtt_med_ms=%s residual_rms_ms=%s "
        "residual_med_ms=%s slope_ns_per_us=%s drift_ppm=%s "
        "map_version=%d invalid=%d",
        response.boot_id,
        response.sync_id,
        quality.sample_count,
        quality.used_sample_count,
        quality.ready,
        ms(write_duration_ns),
        ms((response.t3_send_us - response.t2_receive_us) * 1000),
        ms(latest_rtt_ns),
        ms(quality.rtt_min_ns),
        ms(quality.rtt_median_ns),
        ms(quality.residual_rms_ns),
        ms(quality.residual_median_abs_ns),
        slope,
        drift,
        quality.map_version,
        quality.invalid_count,
    )


def _log_ble_error(exc: BaseException) -> None:
    msg = str(exc).lower()
    if any(token in msg for token in ("pair", "encrypt", "auth", "bond")):
        log.error(
            "BLOCKED: BLE pairing/encryption required. "
            "Firmware must have encrypted=false. (%s)",
            exc,
        )
        return
    log.exception("BLE error: %s", exc)


async def _sleep_until(
    stop_event: threading.Event,
    seconds: float,
    disconnected: asyncio.Event | None = None,
) -> None:
    deadline = time.monotonic() + seconds
    while not stop_event.is_set() and time.monotonic() < deadline:
        if disconnected is not None and disconnected.is_set():
            return
        await asyncio.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
