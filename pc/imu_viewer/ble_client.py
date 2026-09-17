"""BLE central: scan IMU-AHRS / NUS, subscribe TX, feed the protocol decoder.

This module must never import or call VPython. Notify callbacks only write
into PoseStore via ProtocolDecoder.feed().
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from collections.abc import Callable

from imu_viewer.pose_state import PoseStore
from imu_viewer.protocol import ProtocolDecoder, Quaternion

log = logging.getLogger("imu_viewer.ble")

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEFAULT_NAME = "IMU-AHRS"

NotifyPayload = bytes | bytearray | memoryview | str


class NotifyHandler:
    """Shared path for real bleak notifies and fake test callbacks."""

    def __init__(
        self,
        store: PoseStore,
        *,
        print_frames: bool = False,
        on_frame: Callable[[Quaternion], None] | None = None,
    ) -> None:
        self.decoder = ProtocolDecoder()
        self.store = store
        self.print_frames = print_frames
        self.on_frame = on_frame

    def on_notify(self, _sender: object, data: NotifyPayload) -> None:
        frames = self.decoder.feed(_as_bytes(data))
        self.store.ingest(frames, decoder=self.decoder)
        for q in frames:
            if self.print_frames:
                w, x, y, z = q
                print(f"Q,{w:.4f},{x:.4f},{y:.4f},{z:.4f}", flush=True)
            if self.on_frame is not None:
                self.on_frame(q)


class BleClient:
    """Scan by name or NUS UUID, connect, subscribe, reconnect until stopped."""

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
    ) -> None:
        self._store = store
        self._name = name
        self._scan_timeout = scan_timeout
        self._retry_s = retry_s
        self._handler = NotifyHandler(store, print_frames=print_frames)
        self._client = None
        self._loop: asyncio.AbstractEventLoop | None = None

    @property
    def decoder(self) -> ProtocolDecoder:
        return self._handler.decoder

    def handle_notify(self, sender: object, data: NotifyPayload) -> None:
        """bleak notification callback. Must not touch the 3D scene."""
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
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            return service in uuids

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
            self._store.set_connected(True)
            try:
                await client.start_notify(self.TX_UUID, self.handle_notify)
            except Exception as exc:
                _log_ble_error(exc)
                raise
            log.info("subscribed to TX %s", self.TX_UUID)
            while not stop_event.is_set() and not disconnected.is_set():
                if not client.is_connected:
                    break
                await asyncio.sleep(0.1)
        self._client = None
        self._store.set_connected(False)

    async def _disconnect(self) -> None:
        client = self._client
        self._client = None
        if client is not None:
            try:
                if getattr(client, "is_connected", False):
                    await client.disconnect()
            except Exception:
                log.debug("disconnect failed", exc_info=True)
        self._store.set_connected(False)
        log.info("BLE client stopped")


def _as_bytes(data: NotifyPayload) -> bytes:
    if isinstance(data, str):
        return data.encode("utf-8")
    return bytes(data)


def _log_ble_error(exc: BaseException) -> None:
    msg = str(exc).lower()
    if any(token in msg for token in ("pair", "encrypt", "auth", "bond")):
        log.error(
            "BLOCKED: BLE pairing/encryption required. "
            "Agent B must set encrypted=false (FW-06). "
            "Do not bypass pairing in the PC client. (%s)",
            exc,
        )
        return
    log.exception("BLE error: %s", exc)


async def _sleep_until(stop_event: threading.Event, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while not stop_event.is_set() and time.monotonic() < deadline:
        await asyncio.sleep(0.1)
