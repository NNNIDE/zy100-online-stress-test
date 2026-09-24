from __future__ import annotations

import asyncio
import fnmatch
import hashlib
import struct
import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData


LogCallback = Callable[[str], None]
EventCallback = Callable[[dict[str, Any]], None]

# Realtek's reference profile uses ZY100_OTA, while the Factory firmware
# advertises ZY100_FOTA.  Both names identify the same DFU service.  Keep the
# historical singular constant for callers, but accept both on every scan.
DFU_DEVICE_NAME = "ZY100_OTA"
DFU_DEVICE_NAMES = frozenset({"ZY100_OTA", "ZY100_FOTA"})
DFU_SERVICE_UUID = "00006287-3c17-d293-8e48-14fe2e4da212"
DFU_DATA_UUID = "00006387-3c17-d293-8e48-14fe2e4da212"
DFU_CONTROL_POINT_UUID = "00006487-3c17-d293-8e48-14fe2e4da212"

APP_IMAGE_HEADER_SIZE = 1024
DFU_HEADER_SIZE = 12
MP_WRAPPER_HEADER_SIZE = 0x180
IMAGE_BASE_SCAN_LIMIT = 0x400
RTL8762D_APP_IC_TYPE = 0x09
RTL8762D_BANK0_APP_IMAGE_ID = 0x2793

OP_START_DFU = 0x01
OP_RECEIVE_FW_IMAGE_INFO = 0x02
OP_VALIDATE_FW = 0x03
OP_ACTIVATE_IMAGE_RESET = 0x04
OP_REPORT_TARGET_INFO = 0x06
OP_CONNECTION_PARAM_UPDATE = 0x07
OP_BUFFER_CHECK_EN = 0x09
OP_REPORT_BUFFER_CRC = 0x0A

SUCCESS_RESULT_CODES = {0x00, 0x01}
RESPONSE_NOTIFICATION_OPCODES = {0x10, 0x60}

RECEIVE_FW_INFO_SETTLE_DELAY_SECONDS = 0.03
DFU_PRE_VALIDATE_DELAY_SECONDS = 1.0
DFU_SCAN_TIMEOUT_SECONDS = 90.0
BUFFER_CHECK_RETRY_LIMIT = 3
# The Realtek DFU profile requires a connection-parameter update before the
# buffer-check phase.  Older Host builds defined the command but never sent
# it, leaving Windows to choose a conservative connection interval.  Keep a
# short post-command settle only for stacks that apply the update
# asynchronously; this is not a protocol acknowledgement substitute.
DFU_CONNECTION_PARAM_UPDATE_TIMEOUT_SECONDS = 5.0
DFU_BUFFER_CHECK_LINK_SETTLE_DELAY_SECONDS = 0.2
# Data writes use Write Without Response.  A tiny pacing interval avoids
# overflowing older WinRT queues while removing the previous 5 ms artificial
# delay that cost several seconds over a full image.
DFU_BUFFER_CHECK_PACKET_PACING_SECONDS = 0.001


class DfuProtocolError(RuntimeError):
    """Raised when the target returns a DFU-level failure."""


class DfuTransportError(RuntimeError):
    """Raised when BLE transport cannot complete the DFU flow."""


@dataclass(slots=True)
class RealtekAppImage:
    path: str
    file_name: str
    data: bytes
    size_bytes: int
    sha256: str
    image_base: int
    ic_type: int
    secure_version: int
    ctrl_flag: int
    image_id: int
    crc16: int
    payload_len: int

    @property
    def wrapper_size(self) -> int:
        return self.image_base

    @property
    def dfu_data(self) -> bytes:
        return self.data[self.image_base :]

    @property
    def dfu_size_bytes(self) -> int:
        return len(self.dfu_data)

    @property
    def ctrl_header(self) -> bytes:
        return self.data[self.image_base : self.image_base + DFU_HEADER_SIZE]

    @property
    def ctrl_header_end(self) -> int:
        return self.image_base + DFU_HEADER_SIZE

    @property
    def data_start(self) -> int:
        return self.ctrl_header_end


@dataclass(slots=True)
class ImageBaseCandidate:
    image_base: int
    ic_type: int | None
    secure_version: int | None
    ctrl_flag: int | None
    image_id: int | None
    crc16: int | None
    payload_len: int | None
    dfu_size_bytes: int
    expected_dfu_size: int | None
    valid: bool
    reason: str

    def log_line(self) -> str:
        ic = "--" if self.ic_type is None else f"0x{self.ic_type:02X}"
        image_id = "--" if self.image_id is None else f"0x{self.image_id:04X}"
        payload = "--" if self.payload_len is None else f"{self.payload_len} (0x{self.payload_len:08X})"
        expected = "--" if self.expected_dfu_size is None else str(self.expected_dfu_size)
        status = "OK" if self.valid else "FAIL"
        return (
            f"{status} base=0x{self.image_base:X}, ic_type={ic}, image_id={image_id}, "
            f"payload_len={payload}, dfu_size={self.dfu_size_bytes}, "
            f"expected={expected}, reason={self.reason}"
        )


@dataclass(slots=True)
class DfuControlNotification:
    raw: bytes
    notification_opcode: int
    request_opcode: int
    result_code: int
    payload: bytes

    def is_success(self) -> bool:
        return self.result_code in SUCCESS_RESULT_CODES


@dataclass(slots=True)
class DfuCharacteristics:
    data: BleakGATTCharacteristic
    control: BleakGATTCharacteristic
    service_uuid: str


@dataclass(slots=True)
class DfuBufferCheckConfig:
    max_buffer_size: int
    mtu_size: int
    packet_size: int
    window_size: int


def hex_bytes(payload: bytes) -> str:
    return payload.hex(" ").upper()


def normalize_uuid(uuid: str) -> str:
    value = uuid.strip().lower()
    if value.startswith("0x"):
        value = value[2:]
    if len(value) == 4:
        return f"0000{value}-0000-1000-8000-00805f9b34fb"
    return value


def _is_mp_app_image_name(file_name: str) -> bool:
    return fnmatch.fnmatch(file_name.lower(), "app_mp_*.bin")


def _parse_image_base_candidate(data: bytes, image_base: int) -> ImageBaseCandidate:
    file_size = len(data)
    dfu_size = file_size - image_base
    if image_base < 0 or image_base >= file_size:
        return ImageBaseCandidate(
            image_base=image_base,
            ic_type=None,
            secure_version=None,
            ctrl_flag=None,
            image_id=None,
            crc16=None,
            payload_len=None,
            dfu_size_bytes=max(0, dfu_size),
            expected_dfu_size=None,
            valid=False,
            reason="image_base out of file range",
        )
    if image_base + DFU_HEADER_SIZE > file_size:
        return ImageBaseCandidate(
            image_base=image_base,
            ic_type=None,
            secure_version=None,
            ctrl_flag=None,
            image_id=None,
            crc16=None,
            payload_len=None,
            dfu_size_bytes=dfu_size,
            expected_dfu_size=None,
            valid=False,
            reason="ctrl_header out of file range",
        )

    ic_type, secure_version, ctrl_flag, image_id, crc16, payload_len = struct.unpack_from(
        "<BBHHHI", data, image_base
    )
    expected_dfu_size = APP_IMAGE_HEADER_SIZE + payload_len
    failures: list[str] = []
    if ic_type != RTL8762D_APP_IC_TYPE:
        failures.append(f"ic_type 0x{ic_type:02X} != 0x{RTL8762D_APP_IC_TYPE:02X}")
    if image_id != RTL8762D_BANK0_APP_IMAGE_ID:
        failures.append(f"image_id 0x{image_id:04X} != 0x{RTL8762D_BANK0_APP_IMAGE_ID:04X}")
    if payload_len <= 0:
        failures.append("payload_len <= 0")
    if dfu_size != expected_dfu_size:
        failures.append(f"dfu_size {dfu_size} != payload_len + 1024 {expected_dfu_size}")

    return ImageBaseCandidate(
        image_base=image_base,
        ic_type=ic_type,
        secure_version=secure_version,
        ctrl_flag=ctrl_flag,
        image_id=image_id,
        crc16=crc16,
        payload_len=payload_len,
        dfu_size_bytes=dfu_size,
        expected_dfu_size=expected_dfu_size,
        valid=not failures,
        reason="valid" if not failures else "; ".join(failures),
    )


def _find_realtek_app_image_base(data: bytes, file_name: str) -> ImageBaseCandidate:
    attempts: list[ImageBaseCandidate] = []
    attempted_bases: set[int] = set()

    def try_base(image_base: int) -> ImageBaseCandidate:
        attempted_bases.add(image_base)
        candidate = _parse_image_base_candidate(data, image_base)
        attempts.append(candidate)
        return candidate

    if _is_mp_app_image_name(file_name):
        candidate = try_base(MP_WRAPPER_HEADER_SIZE)
        if candidate.valid:
            return candidate

    candidate = try_base(0)
    if candidate.valid:
        return candidate

    scan_limit = min(len(data), IMAGE_BASE_SCAN_LIMIT)
    scan_candidates: list[ImageBaseCandidate] = []
    for image_base in range(0, scan_limit, 4):
        if image_base in attempted_bases:
            continue
        candidate = _parse_image_base_candidate(data, image_base)
        if candidate.valid:
            scan_candidates.append(candidate)

    if len(scan_candidates) == 1:
        return scan_candidates[0]

    lines = [
        "无法识别 Realtek APP image 起始位置。",
        f"file_size={len(data)}, scan_range=file[0:0x{IMAGE_BASE_SCAN_LIMIT:X}], alignment=4",
        "直接尝试：",
        *(f"  {candidate.log_line()}" for candidate in attempts),
    ]
    if scan_candidates:
        lines.append("扫描候选：")
        lines.extend(f"  {candidate.log_line()}" for candidate in scan_candidates)
        lines.append(f"扫描得到 {len(scan_candidates)} 个候选，无法唯一确定 image_base")
    else:
        lines.append("扫描候选：无")
    raise ValueError("\n".join(lines))


def load_realtek_app_image(path: str) -> RealtekAppImage:
    resolved = Path(path).expanduser().resolve()
    if not resolved.exists() or not resolved.is_file():
        raise ValueError("固件文件不存在")

    data = resolved.read_bytes()
    if len(data) < APP_IMAGE_HEADER_SIZE + DFU_HEADER_SIZE:
        raise ValueError("固件文件过小，不是带 1024B APP image header 的 Realtek APP image")

    candidate = _find_realtek_app_image_base(data, resolved.name)

    return RealtekAppImage(
        path=str(resolved),
        file_name=resolved.name,
        data=data,
        size_bytes=len(data),
        sha256=hashlib.sha256(data).hexdigest(),
        image_base=candidate.image_base,
        ic_type=int(candidate.ic_type),
        secure_version=int(candidate.secure_version),
        ctrl_flag=int(candidate.ctrl_flag),
        image_id=int(candidate.image_id),
        crc16=int(candidate.crc16),
        payload_len=int(candidate.payload_len),
    )


def make_command(opcode: int, payload: bytes = b"") -> bytes:
    return bytes([opcode]) + payload


def build_report_target_info(image_id: int) -> bytes:
    return make_command(OP_REPORT_TARGET_INFO, struct.pack("<H", image_id))


def build_connection_param_update(
    *,
    interval_min: int = 6,
    interval_max: int = 12,
    latency: int = 0,
    supervision_timeout: int = 500,
) -> bytes:
    payload = struct.pack("<HHHH", interval_min, interval_max, latency, supervision_timeout)
    return make_command(OP_CONNECTION_PARAM_UPDATE, payload)


def build_buffer_check_enable() -> bytes:
    return make_command(OP_BUFFER_CHECK_EN)


def build_report_buffer_crc(buffer_size: int, crc16: int) -> bytes:
    return make_command(OP_REPORT_BUFFER_CRC, struct.pack("<HH", buffer_size, crc16))


def build_start_dfu(ctrl_header: bytes) -> bytes:
    if len(ctrl_header) != DFU_HEADER_SIZE:
        raise ValueError("Start DFU requires a 12-byte image ctrl header")
    return make_command(OP_START_DFU, ctrl_header + b"\x00\x00\x00\x00")


def build_receive_fw_image_info(image_id: int, cur_offset: int = DFU_HEADER_SIZE) -> bytes:
    return make_command(OP_RECEIVE_FW_IMAGE_INFO, struct.pack("<HI", image_id, cur_offset))


def build_validate_fw(image_id: int) -> bytes:
    return make_command(OP_VALIDATE_FW, struct.pack("<H", image_id))


def build_activate_image_reset() -> bytes:
    return make_command(OP_ACTIVATE_IMAGE_RESET)


def choose_buffer_check_packet_size(mtu_size: int) -> int:
    return (mtu_size - 3) & 0xFFF0


def choose_buffer_check_window_size(max_buffer_size: int, packet_size: int) -> int:
    if packet_size <= 0:
        return 0
    return (max_buffer_size // packet_size) * packet_size


def build_buffer_check_config(max_buffer_size: int, mtu_size: int) -> DfuBufferCheckConfig:
    packet_size = choose_buffer_check_packet_size(mtu_size)
    window_size = choose_buffer_check_window_size(max_buffer_size, packet_size)
    if packet_size <= 0 or window_size <= 0:
        raise DfuProtocolError(
            "Invalid DFU buffer-check parameters: "
            f"max_buffer={max_buffer_size}, mtu={mtu_size}, packet={packet_size}, window={window_size}"
        )
    return DfuBufferCheckConfig(
        max_buffer_size=max_buffer_size,
        mtu_size=mtu_size,
        packet_size=packet_size,
        window_size=window_size,
    )


def realtek_buffer_crc16(data: bytes) -> int:
    crc = 0
    for offset in range(0, len(data), 2):
        if offset + 1 < len(data):
            word = data[offset] | (data[offset + 1] << 8)
        else:
            word = data[offset]
        crc ^= word
    crc &= 0xFFFF
    return ((crc & 0x00FF) << 8) | ((crc & 0xFF00) >> 8)


def parse_control_notification(data: bytes) -> DfuControlNotification:
    if len(data) < 3:
        raise DfuProtocolError(f"DFU notification too short: {hex_bytes(data)}")
    return DfuControlNotification(
        raw=data,
        notification_opcode=data[0],
        request_opcode=data[1],
        result_code=data[2],
        payload=data[3:],
    )


def is_dfu_advertisement(adv: AdvertisementData) -> bool:
    local_name = (adv.local_name or "").strip()
    service_uuids = {normalize_uuid(item) for item in list(adv.service_uuids or [])}
    return local_name in DFU_DEVICE_NAMES or DFU_SERVICE_UUID in service_uuids


def _service_uuid_text(service_uuids: list[str]) -> str:
    if not service_uuids:
        return "<none>"
    return ", ".join(normalize_uuid(item) for item in service_uuids)


def _kv_bytes_hex(items: dict[Any, bytes]) -> str:
    if not items:
        return "<none>"
    return "; ".join(f"{key}: {hex_bytes(bytes(value))}" for key, value in items.items())


def _extract_ad_sections(event_args: Any) -> str:
    try:
        sections = event_args.advertisement.data_sections
    except Exception:
        return "<unavailable>"

    lines: list[str] = []
    try:
        iterator = list(sections)
    except Exception:
        iterator = []

    for section in iterator:
        try:
            data_type = int(getattr(section, "data_type"))
            data = bytes(getattr(section, "data"))
            ad_record = bytes([len(data) + 1, data_type]) + data
            lines.append(hex_bytes(ad_record))
        except Exception:
            continue
    return " | ".join(lines) if lines else "<empty>"


def extract_raw_adv_scan_response(adv: AdvertisementData) -> tuple[str, str]:
    try:
        raw_data = adv.platform_data[1]
    except Exception:
        return "<unavailable>", "<unavailable>"

    adv_raw = "<unavailable>"
    scan_raw = "<unavailable>"
    try:
        raw_adv = getattr(raw_data, "adv", None)
        if raw_adv is not None:
            adv_raw = _extract_ad_sections(raw_adv)
    except Exception:
        pass

    try:
        raw_scan = getattr(raw_data, "scan", None)
        if raw_scan is not None:
            scan_raw = _extract_ad_sections(raw_scan)
    except Exception:
        pass

    return adv_raw, scan_raw


class RealtekDfuOtaClient:
    """Runs the RTL8762D legacy DFU flow on an isolated asyncio loop."""

    def __init__(self, event_callback: EventCallback, log_callback: LogCallback) -> None:
        self._event_callback = event_callback
        self._log_callback = log_callback
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_event_loop,
            name="RealtekDfuOtaThread",
            daemon=True,
        )
        self._thread.start()
        self._future: Future | None = None
        self._notification_queue: asyncio.Queue[bytes] | None = None

    def _run_event_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def start(self, image_path: str) -> bool:
        if self.is_running():
            self._emit({"type": "ota_error", "message": "OTA 正在进行中"})
            return False
        self._future = asyncio.run_coroutine_threadsafe(self._run_upgrade(image_path), self._loop)
        return True

    def is_running(self) -> bool:
        return self._future is not None and not self._future.done()

    def shutdown(self) -> None:
        if self.is_running() and self._future is not None:
            self._future.cancel()
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2)

    def _emit(self, event: dict[str, Any]) -> None:
        self._event_callback(event)

    def _log(self, message: str) -> None:
        self._log_callback(message)

    async def _run_upgrade(self, image_path: str) -> None:
        client: BleakClient | None = None
        disconnected = asyncio.Event()
        validated = False
        activated = False
        completed_event = None

        try:
            image = load_realtek_app_image(image_path)
            self._log_image_info(image)

            self._emit({"type": "ota_status", "state": "SCANNING", "message": "正在 active scan DFU 设备"})
            device, adv = await self._scan_for_dfu_device(timeout=DFU_SCAN_TIMEOUT_SECONDS)
            self._log_scan_match(device, adv)

            self._emit({"type": "ota_status", "state": "CONNECTING", "message": f"正在连接 DFU 设备 {device.address}"})
            client = BleakClient(
                device,
                disconnected_callback=lambda _: self._loop.call_soon_threadsafe(disconnected.set),
            )
            await client.connect()
            self._log(f"DFU connected: address={device.address}")

            try:
                await client.get_services()
            except Exception:
                pass

            chars = self._discover_dfu_characteristics(client)
            self._emit({"type": "ota_status", "state": "CONNECTED", "message": f"DFU 已连接：{device.address}"})

            self._notification_queue = asyncio.Queue()
            await client.start_notify(chars.control, self._handle_control_notification)
            self._log(
                "DFU Control Point notification enabled: "
                f"uuid={chars.control.uuid.lower()}, handle={self._char_handle(chars.control)}"
            )

            # The update is a throughput optimisation, not a DFU prerequisite.
            # Some firmware revisions reject opcode 0x07 (the ARM trace showed
            # result 0x03) even though the DFU service is fully usable.  Treat
            # a rejected/unsupported update as a fallback to the current link
            # parameters so a single optional command cannot strand OTA in
            # PENDING_CONFIRM.  Transport/cancellation errors are also scoped
            # to this optional step; all required DFU commands remain fatal.
            try:
                response = await self._write_control_and_wait(
                    client,
                    chars.control,
                    build_connection_param_update(),
                    OP_CONNECTION_PARAM_UPDATE,
                    "Connection Parameter Update",
                    timeout=DFU_CONNECTION_PARAM_UPDATE_TIMEOUT_SECONDS,
                    required=False,
                    return_failed=True,
                )
                if response is not None and response.is_success():
                    self._log(
                        "DFU connection parameters accepted: "
                        "interval_min=6 (7.5ms), interval_max=12 (15ms), "
                        "latency=0, supervision_timeout=500 (5s)"
                    )
                else:
                    self._log(
                        "WARNING: DFU connection parameter update rejected or "
                        "unsupported; continuing with current link parameters"
                    )
            except Exception as exc:
                self._log(
                    "WARNING: DFU connection parameter update unavailable; "
                    f"continuing with current link parameters: {exc}"
                )
            buffer_config = await self._enable_buffer_check(client, chars.control)

            target_response = await self._write_control_and_wait(
                client,
                chars.control,
                build_report_target_info(image.image_id),
                OP_REPORT_TARGET_INFO,
                "Report Target Info",
            )
            self._log_target_info(target_response)

            self._emit({"type": "ota_status", "state": "TRANSFERRING", "message": "正在启动 DFU"})
            await self._write_control_and_wait(
                client,
                chars.control,
                build_start_dfu(image.ctrl_header),
                OP_START_DFU,
                "Start DFU",
            )
            sent_bytes = DFU_HEADER_SIZE
            self._emit_progress(sent_bytes, image.dfu_size_bytes, "Start DFU")

            await self._write_control_no_notification(
                client,
                chars.control,
                build_receive_fw_image_info(image.image_id, DFU_HEADER_SIZE),
                "Receive FW Image Information",
                settle_delay=RECEIVE_FW_INFO_SETTLE_DELAY_SECONDS,
            )
            if DFU_BUFFER_CHECK_LINK_SETTLE_DELAY_SECONDS > 0:
                self._log(
                    "DFU buffer_check settle wait: "
                    f"{DFU_BUFFER_CHECK_LINK_SETTLE_DELAY_SECONDS:.3f}s before data"
                )
                await asyncio.sleep(DFU_BUFFER_CHECK_LINK_SETTLE_DELAY_SECONDS)

            await self._write_image_data(client, chars.data, chars.control, image, buffer_config, sent_bytes)
            self._log("DFU data transfer complete; waiting 1000ms before Validate FW")
            await asyncio.sleep(DFU_PRE_VALIDATE_DELAY_SECONDS)

            self._emit({"type": "ota_status", "state": "VALIDATING", "message": "正在 Validate FW"})
            await self._write_control_and_wait(
                client,
                chars.control,
                build_validate_fw(image.image_id),
                OP_VALIDATE_FW,
                "Validate FW",
                timeout=20.0,
            )
            validated = True

            self._emit({"type": "ota_status", "state": "ACTIVATING", "message": "正在 Activate Image and Reset"})
            activated = await self._write_activate_reset(client, chars.control, disconnected)

            if not activated:
                raise DfuTransportError("Activate Image and Reset 未确认发送")

            await self._wait_expected_disconnect(client, disconnected, device.address)
            completed_event = {
                    "type": "ota_completed",
                    "validated": validated,
                    "activated": activated,
                    "message": (
                        "OTA 镜像写入及激活已完成，正在等待设备返回并恢复业务连接。"
                    ),
                }
            self._log("DFU OTA completed")
        except asyncio.CancelledError:
            self._emit({"type": "ota_error", "message": "OTA 已取消"})
            self._log("DFU OTA cancelled")
        except Exception as exc:
            self._emit({"type": "ota_error", "message": str(exc)})
            self._log(f"DFU OTA failed: {exc}")
        finally:
            self._notification_queue = None
            if client is not None:
                try:
                    from ble.client_lifecycle import release_client
                    await release_client(client, self._log)
                except Exception as exc:
                    completed_event = None
                    self._emit({"type": "ota_error", "validated": validated, "activated": activated,
                                "message": f"{'镜像已写入，' if validated else ''}恢复资源释放失败：{exc}"})
            if completed_event is not None:
                self._emit(completed_event)

    def _log_image_info(self, image: RealtekAppImage) -> None:
        self._log(f"固件文件：{image.file_name}")
        self._log(f"固件大小：{image.size_bytes} bytes, SHA256={image.sha256}")
        self._log(f"file_size = {image.size_bytes}")
        self._log(f"image_base = 0x{image.image_base:X}")
        self._log(f"wrapper_size = {image.wrapper_size}")
        if image.wrapper_size:
            self._log("wrapper skipped, not sent by DFU")
        self._log(f"dfu_size_bytes = {image.dfu_size_bytes}")
        self._log(f"ctrl_header = file[0x{image.image_base:X}:0x{image.ctrl_header_end:X}]")
        self._log(f"Data start = file[0x{image.data_start:X}:]")
        self._log(
            "APP image ctrl header: "
            f"ic_type=0x{image.ic_type:02X}, secure_version=0x{image.secure_version:02X}, "
            f"ctrl_flag=0x{image.ctrl_flag:04X}, image_id=0x{image.image_id:04X}, "
            f"crc16=0x{image.crc16:04X}, payload_len={image.payload_len} (0x{image.payload_len:08X})"
        )
        self._log(f"APP image ctrl header hex: {hex_bytes(image.ctrl_header)}")

    async def _scan_for_dfu_device(self, timeout: float) -> tuple[BLEDevice, AdvertisementData]:
        found: asyncio.Future[tuple[BLEDevice, AdvertisementData]] = self._loop.create_future()
        seen_candidates: set[str] = set()

        def on_detect(device: BLEDevice, adv: AdvertisementData) -> None:
            if not is_dfu_advertisement(adv):
                return

            key = device.address
            if key not in seen_candidates:
                seen_candidates.add(key)
                self._log_scan_candidate(device, adv)

            if not found.done():
                self._loop.call_soon_threadsafe(self._set_scan_result, found, device, adv)

        scanner = BleakScanner(on_detect, scanning_mode="active")
        self._log(f"Start DFU active scan ({timeout:.1f}s)")
        await scanner.start()
        try:
            return await asyncio.wait_for(found, timeout=timeout)
        finally:
            await scanner.stop()

    @staticmethod
    def _set_scan_result(
        found: asyncio.Future[tuple[BLEDevice, AdvertisementData]],
        device: BLEDevice,
        adv: AdvertisementData,
    ) -> None:
        if not found.done():
            found.set_result((device, adv))

    def _log_scan_candidate(self, device: BLEDevice, adv: AdvertisementData) -> None:
        adv_raw, scan_raw = extract_raw_adv_scan_response(adv)
        self._log(
            "DFU scan candidate: "
            f"address={device.address}, name={(adv.local_name or device.name or '<none>')}, "
            f"rssi={adv.rssi}, services={_service_uuid_text(list(adv.service_uuids or []))}"
        )
        self._log(f"DFU scan manufacturer_data: {_kv_bytes_hex(dict(adv.manufacturer_data or {}))}")
        self._log(f"DFU scan service_data: {_kv_bytes_hex(dict(adv.service_data or {}))}")
        self._log(f"DFU ADV raw: {adv_raw}")
        self._log(f"DFU SCAN_RSP raw: {scan_raw}")

    def _log_scan_match(self, device: BLEDevice, adv: AdvertisementData) -> None:
        name = adv.local_name or device.name or "<none>"
        services = _service_uuid_text(list(adv.service_uuids or []))
        self._log(f"DFU target selected: address={device.address}, name={name}, services={services}")

    def _discover_dfu_characteristics(self, client: BleakClient) -> DfuCharacteristics:
        lines: list[str] = []
        dfu_service: Any | None = None

        for service in client.services:
            service_uuid = normalize_uuid(str(service.uuid))
            lines.append(f"[Service] {service.uuid} | handle={getattr(service, 'handle', '<unknown>')}")
            for char in service.characteristics:
                lines.append(
                    f"    [Char] {char.uuid} | handle={self._char_handle(char)} | props={','.join(char.properties)}"
                )
            if service_uuid == DFU_SERVICE_UUID:
                dfu_service = service

        self._emit({"type": "ota_dfu_services", "lines": lines})
        self._log("DFU service discovery completed")
        for line in lines:
            self._log(line)

        if dfu_service is None:
            raise DfuTransportError(f"未发现 DFU Service: {DFU_SERVICE_UUID}")

        data_char: BleakGATTCharacteristic | None = None
        control_char: BleakGATTCharacteristic | None = None

        for char in dfu_service.characteristics:
            char_uuid = normalize_uuid(str(char.uuid))
            if char_uuid == DFU_DATA_UUID and self._has_property(char, "write-without-response"):
                data_char = char
            if (
                char_uuid == DFU_CONTROL_POINT_UUID
                and self._has_property(char, "write")
                and self._has_property(char, "notify")
            ):
                control_char = char

        if data_char is None:
            for char in dfu_service.characteristics:
                if self._has_property(char, "write-without-response"):
                    data_char = char
                    break

        if control_char is None:
            for char in dfu_service.characteristics:
                if self._has_property(char, "write") and self._has_property(char, "notify"):
                    control_char = char
                    break

        if data_char is None:
            raise DfuTransportError("未找到 DFU Data Characteristic（write without response）")
        if control_char is None:
            raise DfuTransportError("未找到 DFU Control Point Characteristic（write + notify）")

        self._log(
            "DFU Data Characteristic: "
            f"uuid={data_char.uuid.lower()}, handle={self._char_handle(data_char)}, props={','.join(data_char.properties)}"
        )
        self._log(
            "DFU Control Point Characteristic: "
            f"uuid={control_char.uuid.lower()}, handle={self._char_handle(control_char)}, "
            f"props={','.join(control_char.properties)}"
        )
        return DfuCharacteristics(data=data_char, control=control_char, service_uuid=DFU_SERVICE_UUID)

    def _handle_control_notification(self, sender: Any, data: bytearray) -> None:
        payload = bytes(data)
        self._log(f"DFU Control Point notification: sender={sender}, hex={hex_bytes(payload)}")
        queue = self._notification_queue
        if queue is not None:
            self._loop.call_soon_threadsafe(queue.put_nowait, payload)

    async def _write_control_and_wait(
        self,
        client: BleakClient,
        control_char: BleakGATTCharacteristic,
        command: bytes,
        expected_opcode: int,
        label: str,
        *,
        timeout: float = 8.0,
        required: bool = True,
        return_failed: bool = False,
    ) -> DfuControlNotification | None:
        self._log(
            f"DFU CP write {label}: uuid={control_char.uuid.lower()}, "
            f"handle={self._char_handle(control_char)}, hex={hex_bytes(command)}"
        )
        await client.write_gatt_char(control_char, command, response=True)

        queue = self._notification_queue
        if queue is None:
            raise DfuTransportError("DFU notification queue is not ready")

        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                message = f"{label} 等待匹配 notification 超时"
                if required:
                    raise DfuProtocolError(message)
                self._emit({"type": "ota_warning", "message": message})
                self._log(f"WARNING: {message}")
                return None

            try:
                raw = await asyncio.wait_for(queue.get(), timeout=remaining)
            except asyncio.TimeoutError:
                continue

            try:
                parsed = parse_control_notification(raw)
            except DfuProtocolError as exc:
                self._log(f"DFU ignored malformed notification: {exc}")
                continue

            if parsed.request_opcode != expected_opcode:
                self._log(
                    f"DFU ignored notification for another request: expected=0x{expected_opcode:02X}, "
                    f"notification_opcode=0x{parsed.notification_opcode:02X}, "
                    f"request_opcode=0x{parsed.request_opcode:02X}, result=0x{parsed.result_code:02X}"
                )
                continue

            if parsed.notification_opcode not in RESPONSE_NOTIFICATION_OPCODES and parsed.notification_opcode != expected_opcode:
                self._log(
                    f"DFU ignored notification with unexpected notification opcode: "
                    f"expected request=0x{expected_opcode:02X}, "
                    f"notification_opcode=0x{parsed.notification_opcode:02X}"
                )
                continue

            if not parsed.is_success():
                message = (
                    f"{label} 返回失败：notification_opcode=0x{parsed.notification_opcode:02X}, "
                    f"request_opcode=0x{parsed.request_opcode:02X}, result=0x{parsed.result_code:02X}, "
                    f"raw={hex_bytes(parsed.raw)}"
                )
                if return_failed:
                    self._emit({"type": "ota_warning", "message": message})
                    self._log(f"WARNING: {message}")
                    return parsed
                if required:
                    raise DfuProtocolError(message)
                self._emit({"type": "ota_warning", "message": message})
                self._log(f"WARNING: {message}")
                return None

            self._log(
                f"DFU CP response {label}: notification_opcode=0x{parsed.notification_opcode:02X}, "
                f"request_opcode=0x{parsed.request_opcode:02X}, result=0x{parsed.result_code:02X}, "
                f"payload={hex_bytes(parsed.payload) if parsed.payload else '<none>'}"
            )
            return parsed

    async def _enable_buffer_check(
        self,
        client: BleakClient,
        control_char: BleakGATTCharacteristic,
    ) -> DfuBufferCheckConfig:
        response = await self._write_control_and_wait(
            client,
            control_char,
            build_buffer_check_enable(),
            OP_BUFFER_CHECK_EN,
            "Buffer Check Enable",
        )
        if response is None or len(response.payload) < 4:
            raise DfuProtocolError("Buffer Check Enable response missing max_buffer_size and mtu_size")

        max_buffer_size, mtu_size = struct.unpack_from("<HH", response.payload, 0)
        config = build_buffer_check_config(max_buffer_size, mtu_size)
        self._log(
            "DFU buffer_check enabled "
            f"max_buffer={config.max_buffer_size} mtu={config.mtu_size} "
            f"packet={config.packet_size} window={config.window_size}"
        )
        return config

    async def _write_control_no_notification(
        self,
        client: BleakClient,
        control_char: BleakGATTCharacteristic,
        command: bytes,
        label: str,
        *,
        settle_delay: float = 0.0,
    ) -> None:
        self._log(
            f"DFU CP write {label}: uuid={control_char.uuid.lower()}, "
            f"handle={self._char_handle(control_char)}, hex={hex_bytes(command)}"
        )
        await client.write_gatt_char(control_char, command, response=True)
        self._log(
            f"DFU CP submitted {label}: write response received; "
            "no matching notification expected"
        )
        if settle_delay > 0:
            await asyncio.sleep(settle_delay)

    def _log_target_info(self, response: DfuControlNotification | None) -> None:
        if response is None or len(response.payload) < 8:
            self._log("DFU target info payload: <not parsed>")
            return
        origin_image_version, cur_offset = struct.unpack_from("<II", response.payload, 0)
        self._log(
            "DFU target info: "
            f"origin_image_version=0x{origin_image_version:08X}, cur_offset={cur_offset} "
            "(第一阶段仅记录，不做断点续传)"
        )

    async def _write_image_data(
        self,
        client: BleakClient,
        data_char: BleakGATTCharacteristic,
        control_char: BleakGATTCharacteristic,
        image: RealtekAppImage,
        buffer_config: DfuBufferCheckConfig,
        sent_bytes: int,
    ) -> None:
        offset = DFU_HEADER_SIZE
        total = image.dfu_size_bytes
        started_at = time.monotonic()
        last_log_at = started_at
        self._log(
            "DFU data transfer start: "
            f"uuid={data_char.uuid.lower()}, handle={self._char_handle(data_char)}, "
            f"file_offset=0x{image.data_start:X}, dfu_offset={offset}, total={total}, "
            f"packet={buffer_config.packet_size}, window={buffer_config.window_size}"
        )

        retry_count = 0
        window_index = 0
        while offset < total:
            window_start = offset
            window_end = min(total, window_start + buffer_config.window_size)
            crc_payload = image.dfu_data[window_start:window_end]
            if retry_count == 0:
                window_index += 1
            window_started_at = time.monotonic()
            packet_count = 0

            cursor = window_start
            while cursor < window_end:
                packet = image.dfu_data[cursor : min(cursor + buffer_config.packet_size, window_end)]
                await client.write_gatt_char(data_char, packet, response=False)
                cursor += len(packet)
                packet_count += 1
                if DFU_BUFFER_CHECK_PACKET_PACING_SECONDS > 0:
                    await asyncio.sleep(DFU_BUFFER_CHECK_PACKET_PACING_SECONDS)

            report = await self._write_control_and_wait(
                client,
                control_char,
                build_report_buffer_crc(len(crc_payload), realtek_buffer_crc16(crc_payload)),
                OP_REPORT_BUFFER_CRC,
                "Report Buffer CRC",
                required=False,
                return_failed=True,
            )
            report_offset = self._report_buffer_crc_offset(report, window_start)
            if report is not None and report.is_success() and report_offset == window_end:
                offset = report_offset
                sent_bytes = offset
                retry_count = 0
                window_elapsed_ms = (time.monotonic() - window_started_at) * 1000.0
                self._log(
                    "DFU buffer window ok: "
                    f"index={window_index}, start={window_start}, end={window_end}, "
                    f"reported={report_offset}, bytes={len(crc_payload)}, packets={packet_count}, "
                    f"elapsed_ms={window_elapsed_ms:.1f}"
                )
            else:
                retry_count += 1
                if retry_count > BUFFER_CHECK_RETRY_LIMIT:
                    raise DfuTransportError(
                        "Report Buffer CRC failed after "
                        f"{BUFFER_CHECK_RETRY_LIMIT} retries: offset={offset}, "
                        f"window_end={window_end}, last_report_offset={report_offset}"
                    )
                if window_start <= report_offset < window_end:
                    offset = report_offset
                else:
                    offset = window_start
                self._log(
                    "DFU buffer window retry: "
                    f"index={window_index}, attempt={retry_count}/{BUFFER_CHECK_RETRY_LIMIT}, "
                    f"start={window_start}, window_end={window_end}, reported={report_offset}, "
                    f"resume_offset={offset}, packets={packet_count}"
                )
                continue

            now = time.monotonic()
            if now - last_log_at >= 1.0 or offset >= total:
                elapsed = max(0.001, now - started_at)
                speed = sent_bytes / elapsed
                percent = sent_bytes * 100.0 / total
                eta = max(0.0, (total - sent_bytes) / speed)
                self._log(
                    f"DFU data progress: {sent_bytes}/{total} bytes "
                    f"({percent:.1f}%), {speed:.0f} B/s, ETA {eta:.1f}s"
                )
                self._emit_progress(sent_bytes, total, "Receive FW Image", speed=speed, eta=eta)
                last_log_at = now

    @staticmethod
    def _report_buffer_crc_offset(response: DfuControlNotification | None, fallback_offset: int) -> int:
        if response is None:
            return fallback_offset
        if len(response.payload) < 4:
            return fallback_offset
        return struct.unpack_from("<I", response.payload, 0)[0]

    async def _write_activate_reset(
        self,
        client: BleakClient,
        control_char: BleakGATTCharacteristic,
        disconnected: asyncio.Event,
    ) -> bool:
        command = build_activate_image_reset()
        self._log(
            "DFU CP write Activate Image and Reset: "
            f"uuid={control_char.uuid.lower()}, handle={self._char_handle(control_char)}, hex={hex_bytes(command)}"
        )
        try:
            await client.write_gatt_char(control_char, command, response=True)
            self._log("Activate Image and Reset command sent")
            return True
        except Exception as exc:
            try:
                await asyncio.wait_for(disconnected.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                self._log(f"Activate Image and Reset write failed: {exc}")
                return False
            self._log(f"Activate write interrupted by expected disconnect; treating command as sent: {exc}")
            return True

    async def _wait_expected_disconnect(
        self,
        client: BleakClient,
        disconnected: asyncio.Event,
        address: str,
    ) -> None:
        try:
            await asyncio.wait_for(disconnected.wait(), timeout=12.0)
            self._log("DFU device disconnected after Activate Image and Reset (expected)")
            return
        except asyncio.TimeoutError:
            self._log("DFU disconnect not observed within 12s; checking whether DFU advertising disappeared")

        if client.is_connected:
            try:
                await client.disconnect()
            except Exception:
                pass

        if await self._scan_address_or_dfu_seen(address, timeout=4.0):
            raise DfuTransportError("Activate 后仍可扫描到 DFU 广播，未确认设备重启")
        self._log("DFU advertisement disappeared after Activate Image and Reset")

    async def _scan_address_or_dfu_seen(self, address: str, timeout: float) -> bool:
        seen: asyncio.Future[bool] = self._loop.create_future()

        def on_detect(device: BLEDevice, adv: AdvertisementData) -> None:
            if device.address == address or is_dfu_advertisement(adv):
                if not seen.done():
                    seen.set_result(True)

        scanner = BleakScanner(on_detect, scanning_mode="active")
        await scanner.start()
        try:
            try:
                return await asyncio.wait_for(seen, timeout=timeout)
            except asyncio.TimeoutError:
                return False
        finally:
            await scanner.stop()

    def _emit_progress(
        self,
        sent: int,
        total: int,
        phase: str,
        *,
        speed: float | None = None,
        eta: float | None = None,
    ) -> None:
        percent = sent * 100.0 / max(1, total)
        self._emit(
            {
                "type": "ota_progress",
                "phase": phase,
                "sent": sent,
                "total": total,
                "percent": percent,
                "speed": speed,
                "eta": eta,
            }
        )

    @staticmethod
    def _has_property(characteristic: BleakGATTCharacteristic, prop: str) -> bool:
        wanted = prop.lower()
        return any(item.lower() == wanted for item in characteristic.properties)

    @staticmethod
    def _get_mtu_size(client: BleakClient) -> int | None:
        try:
            mtu_size = int(client.mtu_size)
        except Exception:
            return None
        return mtu_size if mtu_size > 0 else None

    @staticmethod
    def _char_handle(characteristic: BleakGATTCharacteristic) -> str:
        handle = getattr(characteristic, "handle", None)
        return f"0x{int(handle):04X}" if isinstance(handle, int) else "<unknown>"
