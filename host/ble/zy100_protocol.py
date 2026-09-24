from __future__ import annotations

import binascii
from .online_stress import (StressRecord, parse_stress_source, parse_stress_status,
                            STATUS as STRESS_STATUS, RECORD_STRESS, FRAME_STRESS)
import struct
import time
from dataclasses import dataclass
from typing import Final


DEVICE_NAME_PREFIX: Final[str] = "ZP-"
PREFERRED_DEVICE_NAME: Final[str] = "ZP-912367"

ZY100_CONTROL_SERVICE_UUID: Final[str] = "9eca0000-4a9d-4f4d-b61a-7d0f5a9a1000"
ZY100_COMMAND_UUID: Final[str] = "9eca0001-4a9d-4f4d-b61a-7d0f5a9a1000"
ZY100_ACK_UUID: Final[str] = "9eca0002-4a9d-4f4d-b61a-7d0f5a9a1000"
ZY100_DEVICE_INFO_UUID: Final[str] = "9eca0003-4a9d-4f4d-b61a-7d0f5a9a1000"
ZY100_EXPORT_DATA_UUID: Final[str] = "9eca0004-4a9d-4f4d-b61a-7d0f5a9a1000"
ZY100_FACTORY_SERVICE_UUID: Final[str] = "9eca0000-4a9d-4f4d-b61a-7d0f5a9a2000"
ZY100_MFG_INFO_UUID: Final[str] = "9eca0001-4a9d-4f4d-b61a-7d0f5a9a2000"
ZY100_DEVICE_CHANNEL_UUID: Final[str] = "9eca0013-4a9d-4f4d-b61a-7d0f5a9a2000"
ZY100_MAX_SIMULTANEOUS_LINKS: Final[int] = 1
ZY100_MAX_PAIRED_CENTRALS: Final[int] = 3

ZY100_COMMAND_SIZE: Final[int] = 20
ZY100_ACK_SIZE: Final[int] = 20
ZY100_EXPORT_FRAME_HEADER_SIZE: Final[int] = 8

CMD_START_CAPTURE: Final[int] = 0x01
CMD_PAUSE_CAPTURE: Final[int] = 0x02
CMD_CLEAR_FLASH: Final[int] = 0x03
CMD_TIME_SYNC: Final[int] = 0x10
CMD_HOST_CI_MODE_ENABLE: Final[int] = 0x12
CMD_HOST_PROFILE_RESULT: Final[int] = 0x13
CMD_GET_LINK_STATE: Final[int] = 0x14
CMD_ENTER_SHIPPING: Final[int] = 0x15
# Kept as an import-only compatibility alias for older test helpers. New
# application code must use CMD_ENTER_SHIPPING.
CMD_HOST_CI_HANDOFF: Final[int] = CMD_ENTER_SHIPPING
CMD_OTA_PREPARE: Final[int] = 0x16
CMD_OTA_COMMIT: Final[int] = 0x17
CMD_OTA_LINK_INTENT: Final[int] = 0x18
CMD_GET_CONNECTION_STATE: Final[int] = 0x19
CMD_CONNECTION_USER_SYNC: Final[int] = 0x1A
CMD_EXPORT_CONFIRM: Final[int] = 0x21
CMD_ONLINE_STREAM_READY: Final[int] = 0x22
CMD_ONLINE_RECORD_ACK: Final[int] = 0x23
CMD_OFFLINE_SESSION_LIST: Final[int] = 0x24
CMD_OFFLINE_SESSION_BEGIN: Final[int] = 0x25
CMD_OFFLINE_CHUNK_ACK: Final[int] = 0x26
CMD_OFFLINE_SESSION_RESUME: Final[int] = 0x27
CMD_OFFLINE_FINAL_CONFIRM: Final[int] = 0x28
CMD_OFFLINE_RECLAIM_STATUS: Final[int] = 0x29
CMD_OFFLINE_FOREIGN_PURGE: Final[int] = 0x2A
CMD_OFFLINE_CAPTURE_START: Final[int] = 0x2B
CMD_OFFLINE_CAPTURE_STOP: Final[int] = 0x2C
CMD_FEATURE_CONFIG_SYNC: Final[int] = 0x2D
CMD_TIMEOUT_ACTION_ACK: Final[int] = 0x1B
CMD_TIMEOUT_ACTION_NOTIFY: Final[int] = 0x7B
TIMEOUT_ACTION_OFFLINE_IDLE: Final[int] = 1
TIMEOUT_ACTION_STANDBY: Final[int] = 2
STATE_DETAIL_LOW_BATTERY_REASON: Final[int] = 0x0A
CMD_CONNECTION_STATE_NOTIFY: Final[int] = 0x7C
CMD_LINK_STATE_NOTIFY: Final[int] = 0x7D
CMD_STATE_NOTIFY: Final[int] = 0x7E
CMD_PING: Final[int] = 0x7F

OTA_DFU_ENTRY_REQUEST: Final[bytes] = bytes.fromhex("44 01 5A 59 4F 54 41")
OTA_DFU_ENTRY_ACK: Final[bytes] = bytes.fromhex("45 01 00")
OTA_DFU_ENTRY_REJECT: Final[bytes] = bytes.fromhex("45 00 00")
OTA_DFU_ENTRY_ACK_TIMEOUT_SECONDS: Final[float] = 3.0
OTA_DFU_ENTRY_DISCONNECT_DELAY_SECONDS: Final[float] = 0.7

COMMAND_NAMES: Final[dict[int, str]] = {
    CMD_START_CAPTURE: "START_CAPTURE",
    CMD_PAUSE_CAPTURE: "PAUSE_CAPTURE",
    CMD_CLEAR_FLASH: "CLEAR_FLASH",
    CMD_TIME_SYNC: "TIME_SYNC",
    CMD_HOST_CI_MODE_ENABLE: "HOST_CI_MODE_ENABLE",
    CMD_HOST_PROFILE_RESULT: "HOST_PROFILE_RESULT",
    CMD_GET_LINK_STATE: "GET_LINK_STATE",
    CMD_ENTER_SHIPPING: "ENTER_SHIPPING",
    CMD_OTA_PREPARE: "OTA_PREPARE",
    CMD_OTA_COMMIT: "OTA_COMMIT",
    CMD_OTA_LINK_INTENT: "OTA_LINK_INTENT",
    CMD_GET_CONNECTION_STATE: "GET_CONNECTION_STATE",
    CMD_CONNECTION_USER_SYNC: "CONNECTION_USER_SYNC",
    CMD_EXPORT_CONFIRM: "EXPORT_CONFIRM",
    CMD_ONLINE_STREAM_READY: "ONLINE_STREAM_READY",
    CMD_ONLINE_RECORD_ACK: "ONLINE_RECORD_ACK",
    CMD_OFFLINE_SESSION_LIST: "OFFLINE_SESSION_LIST",
    CMD_OFFLINE_SESSION_BEGIN: "OFFLINE_SESSION_BEGIN",
    CMD_OFFLINE_CHUNK_ACK: "OFFLINE_CHUNK_ACK",
    CMD_OFFLINE_SESSION_RESUME: "OFFLINE_SESSION_RESUME",
    CMD_OFFLINE_FINAL_CONFIRM: "OFFLINE_FINAL_CONFIRM",
    CMD_OFFLINE_RECLAIM_STATUS: "OFFLINE_RECLAIM_STATUS",
    CMD_OFFLINE_FOREIGN_PURGE: "OFFLINE_FOREIGN_PURGE",
    CMD_OFFLINE_CAPTURE_START: "OFFLINE_CAPTURE_START",
    CMD_OFFLINE_CAPTURE_STOP: "OFFLINE_CAPTURE_STOP",
    CMD_FEATURE_CONFIG_SYNC: "FEATURE_CONFIG_SYNC",
    CMD_TIMEOUT_ACTION_ACK: "TIMEOUT_ACTION_ACK",
    CMD_TIMEOUT_ACTION_NOTIFY: "TIMEOUT_ACTION_NOTIFY",
    CMD_CONNECTION_STATE_NOTIFY: "CONNECTION_STATE_NOTIFY",
    CMD_LINK_STATE_NOTIFY: "LINK_STATE_NOTIFY",
    CMD_STATE_NOTIFY: "STATE_NOTIFY",
    CMD_PING: "PING",
}

HOST_CI_PROTOCOL_VERSION: Final[int] = 3
HOST_CI_PROTOCOL_VERSION_V1: Final[int] = 1
HOST_CI_PROTOCOL_VERSION_V2: Final[int] = 2
HOST_CI_PROTOCOL_VERSION_V3: Final[int] = 3
HOST_PROFILE_MASK_ALL: Final[int] = 0x00000007
HOST_PROFILE_MASK_HYBRID_STANDBY_EXACT: Final[int] = 0x00000008
HOST_PROFILE_SIGNATURE: Final[int] = 0xB56C5520
HOST_CI_ENABLE_DETAIL_CI_SETTLING: Final[int] = 0x2001
HOST_CI_ENABLE_DETAIL_DEVICE_NOT_IDLE: Final[int] = 0x2002
HOST_CI_ENABLE_DETAIL_CAPABILITY_INVALID: Final[int] = 0x2003
HOST_PROFILE_BALANCED: Final[int] = 1
HOST_PROFILE_THROUGHPUT: Final[int] = 2
HOST_PROFILE_POWER: Final[int] = 3
HOST_RESULT_REQUEST_ACCEPTED: Final[int] = 1
HOST_RESULT_REQUEST_FAILED: Final[int] = 2
HOST_RESULT_ACTUAL_STABLE: Final[int] = 3
LINK_STATE_INTENT: Final[int] = 1
LINK_STATE_WAIT_HOST: Final[int] = 2
LINK_STATE_HOST_ACCEPTED: Final[int] = 3
LINK_STATE_APPLIED: Final[int] = 4
LINK_STATE_FAILED: Final[int] = 5
LINK_STATE_TIMEOUT: Final[int] = 6
LINK_STATE_HANDOFF_RELEASE_REQUEST: Final[int] = 7
LINK_STATE_PERIPHERAL_EXACT_PENDING: Final[int] = 8
LINK_STATE_HOST_ACQUIRE_REQUEST: Final[int] = 9
LINK_STATE_HANDOFF_ABORTED: Final[int] = 10

BOOTSTRAP_VERSION_V1: Final[int] = 1
BOOTSTRAP_VERSION_V2: Final[int] = 2
BOOTSTRAP_VERSION_V3: Final[int] = 3
BOOTSTRAP_VERSION_V4: Final[int] = 4
BOOTSTRAP_VERSION: Final[int] = BOOTSTRAP_VERSION_V4
CONNECTION_BOOTSTRAPPING: Final[int] = 0
CONNECTION_CONTROL_READY: Final[int] = 1
CONNECTION_BUSINESS_READY: Final[int] = 2
CONNECTION_RECOVERY_ONLY: Final[int] = 3
CONNECTION_FAILED: Final[int] = 4
CONNECTION_DEFERRED_OFFLINE_CAPTURE: Final[int] = 5
CONNECTION_READY_SECURE: Final[int] = 1 << 0
CONNECTION_READY_ACK_NOTIFY: Final[int] = 1 << 1
CONNECTION_READY_LINK_POLICY: Final[int] = 1 << 2
CONNECTION_READY_CALIBRATION: Final[int] = 1 << 3
CONNECTION_READY_EXPORT_NOTIFY: Final[int] = 1 << 4
CONNECTION_READY_TIME_SYNC: Final[int] = 1 << 5
CONNECTION_READY_OFFLINE_GATE: Final[int] = 1 << 6
CONNECTION_READY_ONLINE_STREAM: Final[int] = 1 << 7
CONNECTION_READY_DEVICE_LOGIC: Final[int] = 1 << 8
CONNECTION_READY_CONTROL: Final[int] = 1 << 9
CONNECTION_READY_BUSINESS: Final[int] = 1 << 10
CONNECTION_READY_OFFLINE_CAPTURE_OBSERVER: Final[int] = 1 << 11
CONNECTION_READY_OFFLINE_SESSION_ATTENTION: Final[int] = 1 << 12
CONNECTION_READY_USER_SYNC: Final[int] = 1 << 13
CONNECTION_READY_FEATURE_CONFIG: Final[int] = 1 << 14
CONNECTION_USER_SYNC_VERSION: Final[int] = 1
HOST_CI_HANDOFF_RELEASE_FOR_EXACT_STANDBY: Final[int] = 1
HOST_CI_HANDOFF_ACQUIRE_WINDOWS_CENTRAL: Final[int] = 2

EXPORT_MAGIC: Final[int] = 0xE7
EXPORT_VERSION: Final[int] = 0x01
EXPORT_START: Final[int] = 0x01
EXPORT_DATA: Final[int] = 0x02
EXPORT_END: Final[int] = 0x03
EXPORT_ABORT: Final[int] = 0x04
EXPORT_FLAG_LAST_FRAME: Final[int] = 0x01

ONLINE_FRAME_START: Final[int] = 0x11
ONLINE_FRAME_RECORD: Final[int] = 0x12
ONLINE_FRAME_END: Final[int] = 0x13
ONLINE_FRAME_ABORT: Final[int] = 0x14
ONLINE_FRAME_TYPES: Final[frozenset[int]] = frozenset(
    {ONLINE_FRAME_START, ONLINE_FRAME_RECORD, ONLINE_FRAME_END, ONLINE_FRAME_ABORT, FRAME_STRESS}
)
ONLINE_RECORD_RAW: Final[int] = 0x01
ONLINE_RECORD_SUMMARY: Final[int] = 0x02
ONLINE_RECORD_EVENT: Final[int] = 0x03
ONLINE_RECORD_END: Final[int] = 0xFF
ONLINE_RECORD_TYPE_NAMES: Final[dict[int, str]] = {
    ONLINE_RECORD_RAW: "RAW",
    ONLINE_RECORD_SUMMARY: "SUMMARY",
    ONLINE_RECORD_EVENT: "EVENT",
    ONLINE_RECORD_END: "END",
    RECORD_STRESS: "STRESS",
}
ONLINE_RECORD_MAGIC: Final[int] = 0x534C4E4F
ONLINE_RECORD_VERSION: Final[int] = 1
ONLINE_RECORD_HEADER_BYTES: Final[int] = 256
ONLINE_RECORD_FRAGMENT_HEADER_BYTES: Final[int] = 28
ONLINE_SPOOL_HEADER_FLAG_SEGMENT_META: Final[int] = 0x01
ONLINE_SPOOL_HEADER_EXTENSION_VERSION: Final[int] = 1
ONLINE_SUMMARY_ACK_BATCH_DEFAULT: Final[int] = 1
ONLINE_CAPABILITY_V1: Final[int] = 0x00000001
ONLINE_CAPABILITY_RTC_ENDPOINT_V1: Final[int] = 0x00000002
ONLINE_CAPABILITY_UNIX_ENDPOINT_V2: Final[int] = 0x00000004
ONLINE_CAPABILITY_FIXED40_UNIX_ENDPOINT_V3: Final[int] = 0x00000008
ONLINE_CAPABILITY_MAG_RAW_100HZ_V1: Final[int] = 0x00000010
ONLINE_CAPABILITY_CURRENT: Final[int] = (
    ONLINE_CAPABILITY_V1
    | ONLINE_CAPABILITY_RTC_ENDPOINT_V1
    | ONLINE_CAPABILITY_UNIX_ENDPOINT_V2
    | ONLINE_CAPABILITY_FIXED40_UNIX_ENDPOINT_V3
    | ONLINE_CAPABILITY_MAG_RAW_100HZ_V1
)
ONLINE_END_BASE_BYTES: Final[int] = 36
ONLINE_END_RTC_META_VERSION_V1: Final[int] = 1
ONLINE_END_RTC_META_BYTES_V1: Final[int] = 44
ONLINE_END_RTC_TOTAL_BYTES_V1: Final[int] = 80
ONLINE_END_RTC_META_VERSION: Final[int] = 2
ONLINE_END_RTC_META_BYTES: Final[int] = 60
ONLINE_END_RTC_TOTAL_BYTES: Final[int] = 96
ONLINE_END_FIXED40_META_VERSION_V3: Final[int] = 3
ONLINE_END_FIXED40_META_BYTES_V3: Final[int] = 32
ONLINE_END_FIXED40_TOTAL_BYTES_V3: Final[int] = 68

FE_RAW_MAGIC: Final[int] = 0x57524546
FE_RAW_CONTINUOUS_VERSION: Final[int] = 2
FE_RAW_IMU_MAG_VERSION: Final[int] = 3
FE_RAW_STATUS_COMMITTED: Final[int] = 1
FE_RAW_FLAG_CONTINUOUS_800HZ: Final[int] = 1 << 6
FE_RAW_FLAG_MAG_RAW_100HZ: Final[int] = 1 << 7
FE_RAW_HEADER_BYTES: Final[int] = 156
FE_RAW_HEADER_PAGE_BYTES: Final[int] = 256
FE_RAW_CONTINUOUS_SECTION_TYPE: Final[int] = 3
FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES: Final[int] = 28
FE_RAW_CONTINUOUS_SAMPLE_HZ: Final[int] = 800
FE_RAW_CONTINUOUS_PACKET_BYTES: Final[int] = 16
FE_RAW_CONTINUOUS_PACKETS_PER_RECORD: Final[int] = 990
FE_RAW_CONTINUOUS_SOURCE_RECORD_BYTES: Final[int] = 16128
FE_RAW_CONTINUOUS_ONLINE_RECORD_BYTES: Final[int] = 16384
FE_RAW_IMU_MAG_ONLINE_RECORD_BYTES: Final[int] = 18176
FE_RAW_MAG_SECTION_TYPE: Final[int] = 4
FE_RAW_MAG_SECTION_HEADER_BYTES: Final[int] = 36
FE_RAW_MAG_SAMPLE_HZ: Final[int] = 100
FE_RAW_MAG_SAMPLE_BYTES: Final[int] = 13
FE_RAW_MAG_RAW_BYTES: Final[int] = 9
FE_RAW_MAG_MAX_SAMPLES_PER_RECORD: Final[int] = 135
FE_RAW_MAG_SECTION_FLAGS_REQUIRED: Final[int] = 0x07
FE_RAW_MAG_SECTION_FLAG_MEAS_M_DONE_GATED: Final[int] = 1 << 3
START_DETAIL_ONLINE_READY: Final[int] = 0x1000
START_DETAIL_ONLINE_FALLBACK_NOT_READY: Final[int] = 0x1001
START_DETAIL_ONLINE_FALLBACK_PREERASE: Final[int] = 0x1002
START_DETAIL_ONLINE_FALLBACK_NOTIFY: Final[int] = 0x1003
START_DETAIL_ONLINE_CI9_NOT_READY: Final[int] = 0x1004
START_DETAIL_ONLINE_MAG_NOT_READY: Final[int] = 0x1005
START_DETAIL_CAL_INFO_UNCONFIRMED: Final[int] = 0x1006
START_DETAIL_ONLINE_CI_LLCP_STUCK: Final[int] = 0x1007

FEUF_MAGIC: Final[int] = 0x46554546
FEUF_VERSION: Final[int] = 1
FEUF_HEADER_BYTES: Final[int] = 36
FEUF_FRAME_MANIFEST: Final[int] = 2
FEUF_FRAME_DATA: Final[int] = 3
FEUF_FRAME_SECTION_END: Final[int] = 4
FEUF_FRAME_EXPORT_END: Final[int] = 5
FEUF_SECTION_MANIFEST: Final[int] = 0
FEUF_SECTION_SESSION_META: Final[int] = 1
FEUF_SECTION_RAW_RECORDS: Final[int] = 2
FEUF_SECTION_SUMMARY_RECORDS: Final[int] = 3
FEUF_SECTION_EVENT_RECORDS: Final[int] = 4
FEUF_SECTION_END: Final[int] = 5
FEUF_SECTION_TRAINING_BEGIN: Final[int] = 0x10
FEUF_SECTION_TRAINING_END: Final[int] = 0x11
FEUF_MANIFEST_V3_MIN_BYTES: Final[int] = 192
FEUF_MANIFEST_V3_CRC_OFFSET: Final[int] = 188
FEUF_PROTOCOL_MAX_PAYLOAD_CAPACITY: Final[int] = 256

DEVICE_STATE_WAIT_START: Final[int] = 0x01
DEVICE_STATE_CAPTURING: Final[int] = 0x02
DEVICE_STATE_STOPPING: Final[int] = 0x03
DEVICE_STATE_CLEARING_FLASH: Final[int] = 0x06
DEVICE_STATE_BLE_EXPORT_WAIT_CONFIRM: Final[int] = 0x0A
DEVICE_STATE_ONLINE_STREAMING: Final[int] = 0x0B
DEVICE_STATE_ONLINE_END_WAIT_ACK: Final[int] = 0x0C
DEVICE_STATE_CALIBRATING: Final[int] = 0x0D
DEVICE_STATE_OFFLINE_INTENT_READY: Final[int] = 0x0E
DEVICE_STATE_OFFLINE_CAPTURING: Final[int] = 0x0F
DEVICE_STATE_OFFLINE_FINALIZING: Final[int] = 0x10
DEVICE_STATE_OFFLINE_SESSION_READY: Final[int] = 0x11
DEVICE_STATE_OFFLINE_SYNCING: Final[int] = 0x12
DEVICE_STATE_OFFLINE_RECLAIMING: Final[int] = 0x13
ZY100_BLE_OFFLINE_DETAIL_START_CANCELLED: Final[int] = 0x3010

STATUS_TEXT: Final[dict[int, str]] = {
    0x00: "OK / 已接收",
    0x01: "BAD_LENGTH",
    0x02: "BAD_MAGIC_OR_VERSION",
    0x03: "UNSUPPORTED_CMD",
    0x04: "BUSY",
    0x05: "NOT_READY",
    0x06: "NOT_IMPLEMENTED",
    0x07: "INVALID_STATE",
    0x08: "INTERNAL_ERROR",
}

DEVICE_STATE_TEXT: Final[dict[int, str]] = {
    0x00: "UNKNOWN / SLEEPING",
    0x01: "AWAKE_IDLE / WAIT_START",
    0x02: "CAPTURING",
    0x03: "STOPPING",
    0x04: "EXPORT_READY",
    0x05: "LEGACY_UART_EXPORTING",
    0x06: "CLEARING_FLASH",
    0x07: "ERROR",
    0x08: "FLASH_FULL_WAIT_CLEAR",
    0x09: "BLE_EXPORTING",
    0x0A: "BLE_EXPORT_WAIT_CONFIRM",
    0x0B: "ONLINE_STREAMING",
    0x0C: "ONLINE_END_WAIT_ACK",
    0x0D: "CALIBRATING",
    0x0E: "OFFLINE_INTENT_READY",
    0x0F: "OFFLINE_CAPTURING",
    0x10: "OFFLINE_FINALIZING",
    0x11: "OFFLINE_SESSION_READY",
    0x12: "OFFLINE_SYNCING",
    0x13: "OFFLINE_RECLAIMING",
}

EXEC_MODE_TEXT: Final[dict[int, str]] = {
    0x00: "NONE",
    0x01: "DRY_RUN_NO_ACTION",
    0x02: "REAL_ACTION / EXECUTED",
    0x03: "ACCEPTED_ASYNC",
    0x04: "ASYNC_DONE",
}


@dataclass(frozen=True, slots=True)
class ZY100Ack:
    cmd_echo: int
    seq_echo: int
    status: int
    device_state: int
    exec_mode: int
    reserved: int
    user_id: int
    training_id: int
    detail: int
    raw: bytes

    def to_dict(self) -> dict[str, int | str | None]:
        return {
            "cmd_echo": self.cmd_echo,
            "cmd_name": command_name(self.cmd_echo),
            "seq_echo": self.seq_echo,
            "status": self.status,
            "status_text": status_text(self.status),
            "device_state": self.device_state,
            "device_state_text": device_state_text(self.device_state),
            "exec_mode": self.exec_mode,
            "exec_mode_text": exec_mode_text(self.exec_mode),
            "reserved": self.reserved,
            "user_id": self.user_id,
            "training_id": self.training_id,
            "detail": self.detail,
            "raw_hex": hex_bytes(self.raw),
        }


@dataclass(frozen=True, slots=True)
class ConnectionStateSnapshot:
    revision: int
    overall_status: int
    device_state: int
    suggested_action: int
    bootstrap_version: int
    generation: int
    ready_bits: int
    stage: int
    reason: int
    retry_ms: int
    raw: bytes


@dataclass(frozen=True, slots=True)
class ExportFrame:
    frame_type: int
    flags: int
    chunk_seq: int
    payload: bytes
    raw: bytes


@dataclass(frozen=True, slots=True)
class ExportStartPayload:
    export_id: int
    session_count: int
    estimated_total_bytes: int
    reserved: int


@dataclass(frozen=True, slots=True)
class ExportEndPayload:
    export_id: int
    total_bytes: int
    stream_crc32: int
    chunk_count: int


@dataclass(frozen=True, slots=True)
class OnlineStartPayload:
    session_id: int
    user_id: int
    training_id: int
    ack_timeout_ms: int
    summary_ack_batch: int
    spool_region_bytes: int
    contiguous_erased_bytes: int

    def to_dict(self) -> dict[str, int | bool]:
        return {
            "session_id": self.session_id,
            "user_id": self.user_id,
            "training_id": self.training_id,
            "ack_timeout_ms": self.ack_timeout_ms,
            "summary_ack_batch": self.summary_ack_batch,
            "spool_region_bytes": self.spool_region_bytes,
            "contiguous_erased_bytes": self.contiguous_erased_bytes,
        }


@dataclass(frozen=True, slots=True)
class OnlineRtcClockMeta:
    version: int
    meta_bytes: int
    supported: bool
    status: int | None
    rtc_nominal_tick_hz: int | None
    accepted_packet_count: int | None
    first_imu_timestamp_raw: int | None
    last_imu_timestamp_raw: int | None
    first_rtc_tick: int | None
    last_rtc_tick: int | None
    rtc_wrap_ticks: int | None
    first_unix_time_ms: int | None
    last_unix_time_ms: int | None
    first_unix_time_us: int | None
    last_unix_time_us: int | None
    raw_hex: str

    def to_dict(self) -> dict[str, int | bool | str | None]:
        return {
            "version": self.version,
            "meta_bytes": self.meta_bytes,
            "supported": self.supported,
            "status": self.status,
            "rtc_nominal_tick_hz": self.rtc_nominal_tick_hz,
            "accepted_packet_count": self.accepted_packet_count,
            "first_imu_timestamp_raw": self.first_imu_timestamp_raw,
            "last_imu_timestamp_raw": self.last_imu_timestamp_raw,
            "first_rtc_tick": self.first_rtc_tick,
            "last_rtc_tick": self.last_rtc_tick,
            "rtc_wrap_ticks": self.rtc_wrap_ticks,
            "first_unix_time_ms": self.first_unix_time_ms,
            "last_unix_time_ms": self.last_unix_time_ms,
            "first_unix_time_us": self.first_unix_time_us,
            "last_unix_time_us": self.last_unix_time_us,
            "raw_hex": self.raw_hex,
        }


@dataclass(frozen=True, slots=True)
class OnlineEndPayload:
    session_id: int
    stop_reason: int
    produced_raw: int
    produced_summary: int
    produced_event: int
    acked_raw: int
    acked_summary: int
    acked_event: int
    pending_high_water: int
    clock_meta: OnlineRtcClockMeta | None = None
    stress: dict | None = None

    def to_dict(self) -> dict[str, object]:
        value: dict[str, object] = {
            "session_id": self.session_id,
            "stop_reason": self.stop_reason,
            "produced_raw": self.produced_raw,
            "produced_summary": self.produced_summary,
            "produced_event": self.produced_event,
            "acked_raw": self.acked_raw,
            "acked_summary": self.acked_summary,
            "acked_event": self.acked_event,
            "pending_high_water": self.pending_high_water,
        }
        value["stress"] = self.stress
        value["clock_meta"] = self.clock_meta.to_dict() if self.clock_meta is not None else None
        return value


@dataclass(frozen=True, slots=True)
class OnlineRecordFragment:
    session_id: int
    record_type: int
    flags: int
    record_id: int
    offset: int
    record_bytes: int
    fragment_len: int
    crc32: int
    data: bytes
    chunk_seq: int

    @property
    def is_last(self) -> bool:
        return (self.flags & EXPORT_FLAG_LAST_FRAME) != 0


@dataclass(frozen=True, slots=True)
class OnlineSpoolHeader:
    magic: int
    version: int
    header_len: int
    session_id: int
    record_type: int
    flags: int
    record_id: int
    payload_bytes: int
    source_record_bytes: int
    crc32: int
    extension_version: int | None = None
    capture_segment_id: int | None = None
    segment_start_offset_ms: int | None = None
    source_id: int | None = None

    @property
    def has_segment_metadata(self) -> bool:
        return (self.flags & ONLINE_SPOOL_HEADER_FLAG_SEGMENT_META) != 0

    def to_dict(self) -> dict[str, int | None]:
        return {
            "magic": self.magic,
            "version": self.version,
            "header_len": self.header_len,
            "session_id": self.session_id,
            "record_type": self.record_type,
            "flags": self.flags,
            "record_id": self.record_id,
            "payload_bytes": self.payload_bytes,
            "source_record_bytes": self.source_record_bytes,
            "crc32": self.crc32,
            "extension_version": self.extension_version,
            "capture_segment_id": self.capture_segment_id,
            "segment_start_offset_ms": self.segment_start_offset_ms,
            "source_id": self.source_id,
        }


@dataclass(frozen=True, slots=True)
class OnlineContinuousRawInfo:
    sample_hz: int
    packet_bytes: int
    packet_count: int
    start_seq: int
    end_seq: int
    first_timestamp_raw: int
    last_timestamp_raw: int
    packet_data_offset: int

    def to_dict(self) -> dict[str, int]:
        return {
            "sample_hz": self.sample_hz,
            "packet_bytes": self.packet_bytes,
            "packet_count": self.packet_count,
            "start_seq": self.start_seq,
            "end_seq": self.end_seq,
            "first_timestamp_raw": self.first_timestamp_raw,
            "last_timestamp_raw": self.last_timestamp_raw,
            "packet_data_offset": self.packet_data_offset,
        }


@dataclass(frozen=True, slots=True)
class OnlineMagRawSample:
    elapsed_us: int
    raw9: bytes

    def to_dict(self) -> dict[str, int | str]:
        return {
            "elapsed_us": self.elapsed_us,
            "raw9_hex": self.raw9.hex(),
        }


@dataclass(frozen=True, slots=True)
class OnlineMagRawInfo:
    sample_hz: int
    sample_bytes: int
    sample_count: int
    section_flags: int
    read_error_count: int
    missed_deadline_count: int
    sample_data_offset: int
    first_elapsed_us: int
    last_elapsed_us: int
    samples: tuple[OnlineMagRawSample, ...]

    @property
    def fresh_ready_gated(self) -> bool:
        return bool(self.section_flags & FE_RAW_MAG_SECTION_FLAG_MEAS_M_DONE_GATED)

    def to_dict(self) -> dict[str, int]:
        return {
            "sample_hz": self.sample_hz,
            "sample_bytes": self.sample_bytes,
            "sample_count": self.sample_count,
            "section_flags": self.section_flags,
            "read_error_count": self.read_error_count,
            "missed_deadline_count": self.missed_deadline_count,
            "sample_data_offset": self.sample_data_offset,
            "first_elapsed_us": self.first_elapsed_us,
            "last_elapsed_us": self.last_elapsed_us,
            "fresh_ready_gated": self.fresh_ready_gated,
        }


@dataclass(frozen=True, slots=True)
class OnlineCompletedRecord:
    session_id: int
    record_type: int
    record_id: int
    record_bytes: int
    payload_bytes: int
    source_record_bytes: int
    crc32: int
    data: bytes
    header: OnlineSpoolHeader
    raw_source_version: int | None = None
    continuous_raw: OnlineContinuousRawInfo | None = None
    mag_raw: OnlineMagRawInfo | None = None
    stress: StressRecord | None = None

    @property
    def record_type_name(self) -> str:
        return online_record_type_name(self.record_type)

    def to_dict(self) -> dict[str, object]:
        return {
            "session_id": self.session_id,
            "record_type": self.record_type,
            "record_type_name": self.record_type_name,
            "record_id": self.record_id,
            "record_bytes": self.record_bytes,
            "payload_bytes": self.payload_bytes,
            "source_record_bytes": self.source_record_bytes,
            "crc32": self.crc32,
            "capture_segment_id": self.header.capture_segment_id,
            "segment_start_offset_ms": self.header.segment_start_offset_ms,
            "source_id": self.header.source_id,
            "raw_source_version": self.raw_source_version,
            "continuous_raw": (
                self.continuous_raw.to_dict() if self.continuous_raw is not None else None
            ),
            "mag_raw": self.mag_raw.to_dict() if self.mag_raw is not None else None,
            "stress": self.stress.to_dict() if self.stress is not None else None,
        }


@dataclass(frozen=True, slots=True)
class OnlineReceiverEvent:
    kind: str
    start: OnlineStartPayload | None = None
    record: OnlineCompletedRecord | None = None
    end: OnlineEndPayload | None = None
    fragment: OnlineRecordFragment | None = None
    stress: dict | None = None
    ok: bool | None = None
    reason: str = ""


@dataclass(frozen=True, slots=True)
class FEUFFrame:
    frame_type: int
    flags: int
    seq: int
    section_id: int
    section_offset: int
    payload: bytes
    payload_crc32: int
    header_crc32: int


@dataclass(frozen=True, slots=True)
class FEUFExportEnd:
    session_id: int
    frames_sent: int
    payload_bytes_sent: int
    section_crc32_xor: int
    status: int

    def to_dict(self) -> dict[str, int]:
        return {
            "session_id": self.session_id,
            "frames_sent": self.frames_sent,
            "payload_bytes_sent": self.payload_bytes_sent,
            "section_crc32_xor": self.section_crc32_xor,
            "status": self.status,
        }


@dataclass(frozen=True, slots=True)
class FEUFManifest:
    protocol_version: int
    session_version: int
    round: int
    stop_reason: int
    live_hz: int
    ois_hz: int
    raw_count: int
    summary_count: int
    event_count: int
    meta_bytes: int
    raw_bytes: int
    summary_bytes: int
    event_bytes: int
    payload_max: int
    total_sections: int
    total_frames_est: int
    total_payload_bytes: int
    manifest_crc32: int
    training_section_count: int
    training_begin_bytes: int
    training_end_bytes: int
    training_flags: int
    section_mask_all: int
    manifest_bytes: int = 108
    header_bytes: int = 108
    session_uid: int = 0
    export_index: int = 0
    export_total: int = 0
    user_id: int = 0
    training_id: int = 0
    session_seq: int = 0
    start_time_ms: int = 0
    end_time_ms: int = 0
    raw_first_bucket: int = 0
    raw_limit_bucket: int = 0
    raw_used_bytes: int = 0
    raw_base_addr: int = 0
    raw_data_begin_addr: int = 0
    raw_data_end_addr: int = 0
    raw_reclaim_end_addr: int = 0
    summary_base_addr: int = 0
    summary_limit_addr: int = 0
    summary_data_begin_addr: int = 0
    summary_data_end_addr: int = 0
    summary_reclaim_end_addr: int = 0
    summary_used_bytes: int = 0
    event_base_addr: int = 0
    event_limit_addr: int = 0
    event_data_begin_addr: int = 0
    event_data_end_addr: int = 0
    event_reclaim_end_addr: int = 0
    event_used_bytes: int = 0
    flags: int = 0
    manifest_stream_crc32: int = 0

    def to_dict(self) -> dict[str, int]:
        return {
            "protocol_version": self.protocol_version,
            "session_version": self.session_version,
            "manifest_bytes": self.manifest_bytes,
            "header_bytes": self.header_bytes,
            "session_uid": self.session_uid,
            "export_index": self.export_index,
            "export_total": self.export_total,
            "user_id": self.user_id,
            "training_id": self.training_id,
            "session_seq": self.session_seq,
            "round": self.round,
            "start_time_ms": self.start_time_ms,
            "end_time_ms": self.end_time_ms,
            "stop_reason": self.stop_reason,
            "live_hz": self.live_hz,
            "ois_hz": self.ois_hz,
            "raw_first_bucket": self.raw_first_bucket,
            "raw_limit_bucket": self.raw_limit_bucket,
            "raw_count": self.raw_count,
            "raw_used_bytes": self.raw_used_bytes,
            "raw_base_addr": self.raw_base_addr,
            "raw_data_begin_addr": self.raw_data_begin_addr,
            "raw_data_end_addr": self.raw_data_end_addr,
            "raw_reclaim_end_addr": self.raw_reclaim_end_addr,
            "summary_base_addr": self.summary_base_addr,
            "summary_limit_addr": self.summary_limit_addr,
            "summary_data_begin_addr": self.summary_data_begin_addr,
            "summary_data_end_addr": self.summary_data_end_addr,
            "summary_reclaim_end_addr": self.summary_reclaim_end_addr,
            "summary_count": self.summary_count,
            "summary_used_bytes": self.summary_used_bytes,
            "event_base_addr": self.event_base_addr,
            "event_limit_addr": self.event_limit_addr,
            "event_data_begin_addr": self.event_data_begin_addr,
            "event_data_end_addr": self.event_data_end_addr,
            "event_reclaim_end_addr": self.event_reclaim_end_addr,
            "event_count": self.event_count,
            "event_used_bytes": self.event_used_bytes,
            "meta_bytes": self.meta_bytes,
            "raw_bytes": self.raw_bytes,
            "summary_bytes": self.summary_bytes,
            "event_bytes": self.event_bytes,
            "payload_max": self.payload_max,
            "total_sections": self.total_sections,
            "total_frames_est": self.total_frames_est,
            "total_payload_bytes": self.total_payload_bytes,
            "manifest_crc32": self.manifest_crc32,
            "training_section_count": self.training_section_count,
            "training_begin_bytes": self.training_begin_bytes,
            "training_end_bytes": self.training_end_bytes,
            "training_flags": self.training_flags,
            "section_mask_all": self.section_mask_all,
            "flags": self.flags,
            "manifest_stream_crc32": self.manifest_stream_crc32,
        }


@dataclass(frozen=True, slots=True)
class TrainingRecord:
    magic: str
    version: int
    record_type: int
    header_len: int
    flags: int
    user_id: int
    training_id: int
    session_seq: int
    capture_round: int
    start_time_ms: int
    end_time_ms: int
    time_source_start: int
    time_source_end: int
    source: int
    stop_reason: int
    data_flags: int
    raw_block_count: int
    summary_count: int
    event_count: int
    crc32: int

    def to_dict(self) -> dict[str, int | str]:
        return {
            "magic": self.magic,
            "version": self.version,
            "record_type": self.record_type,
            "header_len": self.header_len,
            "flags": self.flags,
            "user_id": self.user_id,
            "training_id": self.training_id,
            "session_seq": self.session_seq,
            "capture_round": self.capture_round,
            "start_time_ms": self.start_time_ms,
            "end_time_ms": self.end_time_ms,
            "time_source_start": self.time_source_start,
            "time_source_end": self.time_source_end,
            "source": self.source,
            "stop_reason": self.stop_reason,
            "data_flags": self.data_flags,
            "raw_block_count": self.raw_block_count,
            "summary_count": self.summary_count,
            "event_count": self.event_count,
            "crc32": self.crc32,
        }


@dataclass(frozen=True, slots=True)
class FEUFParseResult:
    frames: tuple[FEUFFrame, ...]
    manifest: FEUFManifest | None
    training_begin: TrainingRecord | None
    training_end: TrainingRecord | None
    export_end: FEUFExportEnd | None
    sections: dict[int, bytes]

    def to_summary_dict(self) -> dict[str, object]:
        manifest_dict = self.manifest.to_dict() if self.manifest else {}
        begin_dict = self.training_begin.to_dict() if self.training_begin else {}
        end_dict = self.training_end.to_dict() if self.training_end else {}
        export_end_dict = self.export_end.to_dict() if self.export_end else {}
        return {
            "frame_count": len(self.frames),
            "section_ids": sorted(self.sections.keys()),
            "manifest": manifest_dict,
            "training_begin": begin_dict,
            "training_end": end_dict,
            "export_end": export_end_dict,
        }


@dataclass(frozen=True, slots=True)
class ExportReceiverEvent:
    kind: str
    start: ExportStartPayload | None = None
    end: ExportEndPayload | None = None
    bytes_received: int = 0
    chunk_count: int = 0
    ok: bool | None = None
    reason: str = ""


class ExportReceiver:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.start: ExportStartPayload | None = None
        self.expected_data_seq = 1
        self.data = bytearray()
        self.stream_crc32 = 0
        self.chunk_count = 0
        self.control_fragments: dict[tuple[int, int], bytearray] = {}
        self.receiving = False
        self.failed = False
        self.fail_reason = ""
        self.end_received = False

    def on_notify(self, data: bytes | bytearray) -> ExportReceiverEvent | None:
        frame = parse_export_frame(data)

        if frame.frame_type in (EXPORT_START, EXPORT_END):
            payload = self._collect_control_payload(frame)
            if payload is None:
                return None
            if frame.frame_type == EXPORT_START:
                start = parse_export_start_payload(payload)
                self.reset()
                self.start = start
                self.receiving = True
                return ExportReceiverEvent(kind="start", start=start)
            end = parse_export_end_payload(payload)
            self.end_received = True
            if self.failed:
                return ExportReceiverEvent(
                    kind="end",
                    end=end,
                    bytes_received=len(self.data),
                    chunk_count=self.chunk_count,
                    ok=False,
                    reason=self.fail_reason,
                )
            ok, reason = self._validate_end(end)
            return ExportReceiverEvent(
                kind="end",
                end=end,
                bytes_received=len(self.data),
                chunk_count=self.chunk_count,
                ok=ok,
                reason=reason,
            )

        if frame.frame_type == EXPORT_DATA:
            return self._handle_data(frame)

        if frame.frame_type == EXPORT_ABORT:
            self.failed = True
            self.fail_reason = "device sent EXPORT_ABORT"
            return ExportReceiverEvent(kind="abort", ok=False, reason=self.fail_reason)

        self.failed = True
        self.fail_reason = f"unknown export frame_type {frame.frame_type}"
        return ExportReceiverEvent(kind="failed", ok=False, reason=self.fail_reason)

    def _collect_control_payload(self, frame: ExportFrame) -> bytes | None:
        key = (frame.frame_type, frame.chunk_seq)
        buf = self.control_fragments.setdefault(key, bytearray())
        buf.extend(frame.payload)
        if (frame.flags & EXPORT_FLAG_LAST_FRAME) == 0:
            return None
        payload = bytes(buf)
        del self.control_fragments[key]
        if len(payload) != 16:
            raise ValueError(f"bad control payload length: {len(payload)}")
        return payload

    def _handle_data(self, frame: ExportFrame) -> ExportReceiverEvent:
        if not self.receiving or self.start is None:
            self.failed = True
            self.fail_reason = "DATA before EXPORT_START"
            return ExportReceiverEvent(kind="failed", ok=False, reason=self.fail_reason)
        if self.failed:
            return ExportReceiverEvent(
                kind="data_ignored",
                bytes_received=len(self.data),
                chunk_count=self.chunk_count,
                ok=False,
                reason=self.fail_reason,
            )
        if frame.chunk_seq != self.expected_data_seq:
            self.failed = True
            self.fail_reason = f"chunk seq mismatch got={frame.chunk_seq} expected={self.expected_data_seq}"
            return ExportReceiverEvent(
                kind="failed",
                bytes_received=len(self.data),
                chunk_count=self.chunk_count,
                ok=False,
                reason=self.fail_reason,
            )

        self.data.extend(frame.payload)
        self.stream_crc32 = binascii.crc32(frame.payload, self.stream_crc32) & 0xFFFFFFFF
        self.chunk_count += 1
        self.expected_data_seq += 1
        return ExportReceiverEvent(kind="data", bytes_received=len(self.data), chunk_count=self.chunk_count)

    def _validate_end(self, end: ExportEndPayload) -> tuple[bool, str]:
        if self.start is None:
            return False, "EXPORT_END before EXPORT_START"
        if end.export_id != self.start.export_id:
            return False, f"export_id mismatch got={end.export_id} expected={self.start.export_id}"
        if end.total_bytes != len(self.data):
            return False, f"total_bytes mismatch got={end.total_bytes} expected={len(self.data)}"
        if end.stream_crc32 != self.stream_crc32:
            return False, f"stream_crc32 mismatch got=0x{end.stream_crc32:08X} expected=0x{self.stream_crc32:08X}"
        if end.chunk_count != self.chunk_count:
            return False, f"chunk_count mismatch got={end.chunk_count} expected={self.chunk_count}"
        return True, ""

    def get_stream(self) -> bytes:
        return bytes(self.data)


class OnlineStreamReceiver:
    def __init__(self) -> None:
        self.stress_enabled = False
        self.reset()

    def reset(self) -> None:
        self.start: OnlineStartPayload | None = None
        self.end: OnlineEndPayload | None = None
        self.failed = False
        self.fail_reason = ""
        self._assembly: dict[tuple[int, int, int], bytearray] = {}
        self._expected_offsets: dict[tuple[int, int, int], int] = {}

    def on_notify(self, data: bytes | bytearray) -> OnlineReceiverEvent | None:
        frame = parse_export_frame(data)
        if frame.frame_type == ONLINE_FRAME_START:
            start = parse_online_start_payload(frame.payload)
            self.reset()
            self.start = start
            return OnlineReceiverEvent(kind="start", start=start, ok=True)
        if frame.frame_type == ONLINE_FRAME_RECORD:
            return self._handle_record_frame(frame)
        if frame.frame_type == FRAME_STRESS:
            if not self.stress_enabled or self.start is None:
                raise ValueError("STRESS status without negotiated session")
            status = parse_stress_status(frame.payload, self.start.session_id)
            return OnlineReceiverEvent(kind="stress", stress=status, ok=True)
        if frame.frame_type == ONLINE_FRAME_END:
            end = parse_online_end_payload(frame.payload)
            if end.stress is not None and not self.stress_enabled:
                raise ValueError("STRESS END without negotiation")
            self.end = end
            return OnlineReceiverEvent(kind="end", end=end, ok=True)
        if frame.frame_type == ONLINE_FRAME_ABORT:
            self.failed = True
            self.fail_reason = "device sent ONLINE_ABORT"
            stress = None
            if frame.payload:
                if not self.stress_enabled or self.start is None:
                    raise ValueError("STRESS ABORT without negotiated session")
                stress = parse_stress_status(frame.payload, self.start.session_id)
            return OnlineReceiverEvent(kind="abort", ok=False, reason=self.fail_reason, stress=stress)
        return None

    def _handle_record_frame(self, frame: ExportFrame) -> OnlineReceiverEvent:
        if self.start is None:
            self.failed = True
            self.fail_reason = "ONLINE_RECORD before ONLINE_START"
            return OnlineReceiverEvent(kind="failed", ok=False, reason=self.fail_reason)
        fragment = parse_online_record_fragment(frame)
        if fragment.record_type == RECORD_STRESS and not self.stress_enabled:
            raise ValueError("STRESS record without negotiation")
        if fragment.session_id != self.start.session_id:
            self.failed = True
            self.fail_reason = (
                f"online session_id mismatch got={fragment.session_id} expected={self.start.session_id}"
            )
            return OnlineReceiverEvent(kind="failed", fragment=fragment, ok=False, reason=self.fail_reason)

        key = (fragment.session_id, fragment.record_type, fragment.record_id)
        expected_offset = self._expected_offsets.get(key, 0)
        if fragment.offset != expected_offset:
            self.failed = True
            self.fail_reason = (
                f"online record offset mismatch type={fragment.record_type} id={fragment.record_id} "
                f"got={fragment.offset} expected={expected_offset}"
            )
            self._assembly.pop(key, None)
            self._expected_offsets.pop(key, None)
            return OnlineReceiverEvent(kind="failed", fragment=fragment, ok=False, reason=self.fail_reason)

        buf = self._assembly.setdefault(key, bytearray())
        buf.extend(fragment.data)
        self._expected_offsets[key] = fragment.offset + fragment.fragment_len
        if not fragment.is_last:
            return OnlineReceiverEvent(kind="fragment", fragment=fragment, ok=True)

        record_data = bytes(buf)
        self._assembly.pop(key, None)
        self._expected_offsets.pop(key, None)
        try:
            record = validate_online_completed_record(fragment, record_data)
        except ValueError as exc:
            self.failed = True
            self.fail_reason = str(exc)
            return OnlineReceiverEvent(kind="failed", fragment=fragment, ok=False, reason=self.fail_reason)
        return OnlineReceiverEvent(kind="record", record=record, fragment=fragment, ok=True)


def command_name(cmd: int) -> str:
    return COMMAND_NAMES.get(cmd & 0xFF, f"UNKNOWN_0x{cmd & 0xFF:02X}")


def status_text(status: int) -> str:
    return STATUS_TEXT.get(status & 0xFF, f"UNKNOWN_0x{status & 0xFF:02X}")


def device_state_text(state: int) -> str:
    return DEVICE_STATE_TEXT.get(state & 0xFF, f"UNKNOWN_0x{state & 0xFF:02X}")


def exec_mode_text(mode: int) -> str:
    return EXEC_MODE_TEXT.get(mode & 0xFF, f"UNKNOWN_0x{mode & 0xFF:02X}")


def hex_bytes(data: bytes | bytearray) -> str:
    return bytes(data).hex(" ").upper()


def online_record_type_name(record_type: int) -> str:
    return ONLINE_RECORD_TYPE_NAMES.get(record_type & 0xFF, f"UNKNOWN_0x{record_type & 0xFF:02X}")


def is_online_export_frame_type(frame_type: int) -> bool:
    return (frame_type & 0xFF) in ONLINE_FRAME_TYPES


def build_zy100_command(
    cmd: int,
    seq: int,
    user_id: int = 1,
    training_id: int = 1,
    device_time_ms: int | None = None,
) -> bytes:
    if device_time_ms is None:
        device_time_ms = int(time.time() * 1000)

    payload = struct.pack(
        "<BBBBIQI",
        0xA5,
        0x01,
        cmd & 0xFF,
        seq & 0xFF,
        user_id & 0xFFFFFFFF,
        device_time_ms & 0xFFFFFFFFFFFFFFFF,
        training_id & 0xFFFFFFFF,
    )
    if len(payload) != ZY100_COMMAND_SIZE:
        raise RuntimeError(f"Unexpected command size: {len(payload)}")
    return payload


def parse_timeout_action(ack: ZY100Ack) -> dict[str, int]:
    if (ack.cmd_echo != CMD_TIMEOUT_ACTION_NOTIFY or ack.status != 0
            or ack.reserved != 0 or ack.exec_mode != 1
            or ack.detail not in {TIMEOUT_ACTION_OFFLINE_IDLE, TIMEOUT_ACTION_STANDBY}
            or ack.training_id == 0 or ack.seq_echo != (ack.training_id & 0xFF)):
        raise ValueError("invalid timeout-action notification")
    return {"generation": ack.user_id, "token": ack.training_id,
            "reason": ack.detail, "seq": ack.seq_echo}


def build_timeout_action_ack(generation: int, token: int, reason: int) -> bytes:
    if not (0 <= generation <= 0xFFFFFFFF and 0 < token <= 0xFFFFFFFF
            and reason in {TIMEOUT_ACTION_OFFLINE_IDLE, TIMEOUT_ACTION_STANDBY}):
        raise ValueError("invalid timeout-action confirmation")
    return build_zy100_command(CMD_TIMEOUT_ACTION_ACK, seq=token & 0xFF,
                               user_id=generation, training_id=token,
                               device_time_ms=reason)


def build_host_ci_mode_enable_command(
    seq: int,
    session_id: int,
    windows_build: int,
    profile_signature: int = HOST_PROFILE_SIGNATURE,
    profile_mask: int = HOST_PROFILE_MASK_ALL,
    protocol_version: int = HOST_CI_PROTOCOL_VERSION,
) -> bytes:
    packed_runtime = (
        (windows_build & 0xFFFFFFFF)
        | ((profile_signature & 0xFFFFFFFF) << 32)
    )
    return build_zy100_command(
        CMD_HOST_CI_MODE_ENABLE,
        seq=seq,
        user_id=session_id,
        training_id=(profile_mask & 0x00FFFFFF)
        | ((protocol_version & 0xFF) << 24),
        device_time_ms=packed_runtime,
    )


def build_host_profile_result_command(
    seq: int,
    session_id: int,
    generation: int,
    transition_id: int,
    profile: int,
    request_status: int,
    result: int,
) -> bytes:
    packed_transition = (
        (generation & 0xFFFFFFFF)
        | ((transition_id & 0xFFFFFFFF) << 32)
    )
    packed_result = (
        (profile & 0xFF)
        | ((request_status & 0xFF) << 8)
        | ((result & 0xFF) << 16)
    )
    return build_zy100_command(
        CMD_HOST_PROFILE_RESULT,
        seq=seq,
        user_id=session_id,
        training_id=packed_result,
        device_time_ms=packed_transition,
    )


def build_host_ci_handoff_command(
    seq: int,
    session_id: int,
    generation: int,
    transition_id: int,
    action: int,
    target_state: int,
) -> bytes:
    packed_transition = (
        (generation & 0xFFFFFFFF)
        | ((transition_id & 0xFFFFFFFF) << 32)
    )
    packed_action = (action & 0xFF) | ((target_state & 0xFF) << 8)
    return build_zy100_command(
        CMD_ENTER_SHIPPING,
        seq=seq,
        user_id=session_id,
        training_id=packed_action,
        device_time_ms=packed_transition,
    )


def build_enter_shipping_command(
    seq: int,
    user_id: int,
    device_time_ms: int | None = None,
) -> bytes:
    """Build the ordinary Production Control ENTER_SHIPPING command.

    arg1 carries the host time for the fixed 20-byte frame; arg2 is reserved.
    The firmware does not use either value to authorize the transition.
    """
    return build_zy100_command(
        CMD_ENTER_SHIPPING,
        seq=seq,
        user_id=user_id,
        training_id=0,
        device_time_ms=device_time_ms,
    )


def build_get_link_state_command(seq: int, session_id: int) -> bytes:
    return build_zy100_command(
        CMD_GET_LINK_STATE,
        seq=seq,
        user_id=session_id,
        training_id=0,
        device_time_ms=0,
    )


def build_get_connection_state_command(seq: int) -> bytes:
    return build_zy100_command(
        CMD_GET_CONNECTION_STATE,
        seq=seq,
        user_id=0,
        training_id=0,
        device_time_ms=0,
    )


def build_connection_user_sync_command(seq: int, user_id: int) -> bytes:
    if not 1 <= user_id <= 0xFFFFFFFF:
        raise ValueError("user_id must be 1..0xFFFFFFFF")
    return build_zy100_command(
        CMD_CONNECTION_USER_SYNC,
        seq=seq,
        user_id=user_id,
        training_id=0,
        device_time_ms=0,
    )


def build_feature_config_sync_command(
    seq: int,
    user_id: int,
    config_bytes: bytes,
) -> bytes:
    if not 1 <= user_id <= 0xFFFFFFFF:
        raise ValueError("user_id must be 1..0xFFFFFFFF")
    if len(config_bytes) != 8:
        raise ValueError("feature config must be exactly 8 bytes")
    config_crc32 = binascii.crc32(
        struct.pack("<I", user_id) + config_bytes
    ) & 0xFFFFFFFF
    return build_zy100_command(
        CMD_FEATURE_CONFIG_SYNC,
        seq=seq,
        user_id=user_id,
        training_id=config_crc32,
        device_time_ms=int.from_bytes(config_bytes, "little"),
    )


def build_ota_prepare_command(seq: int, session_id: int) -> bytes:
    return build_zy100_command(
        CMD_OTA_PREPARE,
        seq=seq,
        user_id=session_id,
        training_id=0,
        device_time_ms=0,
    )


def build_ota_link_intent_command(seq: int, session_id: int) -> bytes:
    return build_zy100_command(
        CMD_OTA_LINK_INTENT,
        seq=seq,
        user_id=session_id,
        training_id=0,
        device_time_ms=0,
    )


def build_ota_commit_command(seq: int, session_id: int) -> bytes:
    return build_zy100_command(
        CMD_OTA_COMMIT,
        seq=seq,
        user_id=session_id,
        training_id=0,
        device_time_ms=0,
    )


def build_time_sync_command(
    seq: int,
    user_id: int = 1,
    training_id: int = 1,
    unix_time_ms: int | None = None,
) -> bytes:
    if unix_time_ms is None:
        unix_time_ms = int(time.time() * 1000)
    if not 1 <= user_id <= 0xFFFFFFFF:
        raise ValueError("user_id must be 1..0xFFFFFFFF")
    if training_id <= 0:
        training_id = 1

    return build_zy100_command(
        CMD_TIME_SYNC,
        seq=seq,
        user_id=user_id,
        training_id=training_id,
        device_time_ms=unix_time_ms,
    )


def build_export_confirm_command(
    seq: int,
    export_id: int,
    result: int,
    host_time_ms: int | None = None,
) -> bytes:
    if host_time_ms is None:
        host_time_ms = int(time.time() * 1000)
    return build_zy100_command(
        CMD_EXPORT_CONFIRM,
        seq=seq,
        user_id=export_id,
        training_id=result,
        device_time_ms=host_time_ms,
    )


def build_online_stream_ready_command(
    seq: int,
    *,
    capability_mask: int = ONLINE_CAPABILITY_CURRENT,
    host_time_ms: int | None = None,
) -> bytes:
    if host_time_ms is None:
        host_time_ms = int(time.time() * 1000)
    return build_zy100_command(
        CMD_ONLINE_STREAM_READY,
        seq=seq,
        user_id=capability_mask,
        training_id=0,
        device_time_ms=host_time_ms,
    )


def build_online_record_ack_command(
    seq: int,
    *,
    session_id: int,
    record_type: int,
    record_id: int,
    ack_count: int = 1,
    status: int = 0,
) -> bytes:
    ack_info = ((status & 0xFF) << 16) | ((ack_count & 0xFF) << 8) | (record_type & 0xFF)
    return build_zy100_command(
        CMD_ONLINE_RECORD_ACK,
        seq=seq,
        user_id=session_id,
        training_id=ack_info,
        device_time_ms=record_id,
    )


def parse_export_frame(data: bytes | bytearray) -> ExportFrame:
    payload = bytes(data)
    if len(payload) < ZY100_EXPORT_FRAME_HEADER_SIZE:
        raise ValueError(f"export frame too short: {len(payload)}")

    magic, version, frame_type, flags, chunk_seq, payload_len = struct.unpack("<BBBBHH", payload[:8])
    if magic != EXPORT_MAGIC:
        raise ValueError(f"export magic error: 0x{magic:02X}")
    if version != EXPORT_VERSION:
        raise ValueError(f"export version error: 0x{version:02X}")
    if payload_len != len(payload) - ZY100_EXPORT_FRAME_HEADER_SIZE:
        raise ValueError(f"export payload length mismatch: {payload_len} != {len(payload) - 8}")

    return ExportFrame(
        frame_type=frame_type,
        flags=flags,
        chunk_seq=chunk_seq,
        payload=payload[ZY100_EXPORT_FRAME_HEADER_SIZE:],
        raw=payload,
    )


def parse_export_start_payload(payload: bytes | bytearray) -> ExportStartPayload:
    data = bytes(payload)
    if len(data) != 16:
        raise ValueError(f"EXPORT_START payload length error: {len(data)}")
    export_id, session_count, estimated_total_bytes, reserved = struct.unpack("<IIII", data)
    return ExportStartPayload(
        export_id=export_id,
        session_count=session_count,
        estimated_total_bytes=estimated_total_bytes,
        reserved=reserved,
    )


def parse_export_end_payload(payload: bytes | bytearray) -> ExportEndPayload:
    data = bytes(payload)
    if len(data) != 16:
        raise ValueError(f"EXPORT_END payload length error: {len(data)}")
    export_id, total_bytes, stream_crc32, chunk_count = struct.unpack("<IIII", data)
    return ExportEndPayload(
        export_id=export_id,
        total_bytes=total_bytes,
        stream_crc32=stream_crc32,
        chunk_count=chunk_count,
    )


def parse_online_start_payload(payload: bytes | bytearray) -> OnlineStartPayload:
    data = bytes(payload)
    if len(data) != 28:
        raise ValueError(f"ONLINE_START payload length error: {len(data)}")
    (
        session_id,
        user_id,
        training_id,
        ack_timeout_ms,
        summary_ack_batch,
        spool_region_bytes,
        contiguous_erased_bytes,
    ) = struct.unpack("<IIIIIII", data)
    return OnlineStartPayload(
        session_id=session_id,
        user_id=user_id,
        training_id=training_id,
        ack_timeout_ms=ack_timeout_ms,
        summary_ack_batch=summary_ack_batch,
        spool_region_bytes=spool_region_bytes,
        contiguous_erased_bytes=contiguous_erased_bytes,
    )


def parse_online_end_payload(payload: bytes | bytearray) -> OnlineEndPayload:
    data = bytes(payload)
    stress = None
    if len(data) == ONLINE_END_FIXED40_TOTAL_BYTES_V3 + STRESS_STATUS.size:
        stress = parse_stress_status(data[ONLINE_END_FIXED40_TOTAL_BYTES_V3:],
                                     struct.unpack_from("<I", data)[0])
        data = data[:ONLINE_END_FIXED40_TOTAL_BYTES_V3]
    if len(data) != ONLINE_END_BASE_BYTES and len(data) < 40:
        raise ValueError(f"ONLINE_END payload length error: {len(data)}")
    (
        session_id,
        stop_reason,
        produced_raw,
        produced_summary,
        produced_event,
        acked_raw,
        acked_summary,
        acked_event,
        pending_high_water,
    ) = struct.unpack("<IIIIIIIII", data[:ONLINE_END_BASE_BYTES])
    clock_meta: OnlineRtcClockMeta | None = None
    if len(data) > ONLINE_END_BASE_BYTES:
        version, meta_bytes = struct.unpack_from("<HH", data, ONLINE_END_BASE_BYTES)
        raw_hex = data[ONLINE_END_BASE_BYTES:].hex()
        if version == ONLINE_END_FIXED40_META_VERSION_V3:
            if meta_bytes != ONLINE_END_FIXED40_META_BYTES_V3:
                raise ValueError(
                    f"ONLINE_END Fixed40 metadata length error: {meta_bytes}"
                )
            if len(data) != ONLINE_END_FIXED40_TOTAL_BYTES_V3:
                raise ValueError(
                    f"ONLINE_END Fixed40 metadata payload length error: {len(data)}"
                )
            (
                status,
                accepted_packet_count,
                first_imu_timestamp_raw,
                last_imu_timestamp_raw,
                first_unix_time_us,
                last_unix_time_us,
            ) = struct.unpack_from("<IIHHQQ", data, 40)
            clock_meta = OnlineRtcClockMeta(
                version=version,
                meta_bytes=meta_bytes,
                supported=True,
                status=status,
                rtc_nominal_tick_hz=None,
                accepted_packet_count=accepted_packet_count,
                first_imu_timestamp_raw=first_imu_timestamp_raw,
                last_imu_timestamp_raw=last_imu_timestamp_raw,
                first_rtc_tick=None,
                last_rtc_tick=None,
                rtc_wrap_ticks=None,
                first_unix_time_ms=None,
                last_unix_time_ms=None,
                first_unix_time_us=first_unix_time_us,
                last_unix_time_us=last_unix_time_us,
                raw_hex=raw_hex,
            )
        elif version in (ONLINE_END_RTC_META_VERSION_V1, ONLINE_END_RTC_META_VERSION):
            expected_meta_bytes = (
                ONLINE_END_RTC_META_BYTES_V1
                if version == ONLINE_END_RTC_META_VERSION_V1
                else ONLINE_END_RTC_META_BYTES
            )
            expected_total_bytes = ONLINE_END_BASE_BYTES + expected_meta_bytes
            if meta_bytes != expected_meta_bytes:
                raise ValueError(
                    f"ONLINE_END RTC metadata length error: {meta_bytes}"
                )
            if len(data) != expected_total_bytes:
                raise ValueError(
                    f"ONLINE_END RTC metadata payload length error: {len(data)}"
                )
            (
                status,
                rtc_nominal_tick_hz,
                accepted_packet_count,
                first_imu_timestamp_raw,
                last_imu_timestamp_raw,
                first_rtc_tick,
                last_rtc_tick,
                rtc_wrap_ticks,
            ) = struct.unpack_from("<IIIHHQQQ", data, 40)
            first_unix_time_ms = None
            last_unix_time_ms = None
            if version == ONLINE_END_RTC_META_VERSION:
                first_unix_time_ms, last_unix_time_ms = struct.unpack_from(
                    "<QQ", data, 80
                )
            clock_meta = OnlineRtcClockMeta(
                version=version,
                meta_bytes=meta_bytes,
                supported=True,
                status=status,
                rtc_nominal_tick_hz=rtc_nominal_tick_hz,
                accepted_packet_count=accepted_packet_count,
                first_imu_timestamp_raw=first_imu_timestamp_raw,
                last_imu_timestamp_raw=last_imu_timestamp_raw,
                first_rtc_tick=first_rtc_tick,
                last_rtc_tick=last_rtc_tick,
                rtc_wrap_ticks=rtc_wrap_ticks,
                first_unix_time_ms=first_unix_time_ms,
                last_unix_time_ms=last_unix_time_ms,
                first_unix_time_us=None,
                last_unix_time_us=None,
                raw_hex=raw_hex,
            )
        else:
            expected_total_bytes = ONLINE_END_BASE_BYTES + meta_bytes
            if len(data) != expected_total_bytes:
                raise ValueError(
                    f"ONLINE_END unknown metadata payload length error: {len(data)}"
                )
            clock_meta = OnlineRtcClockMeta(
                version=version,
                meta_bytes=meta_bytes,
                supported=False,
                status=None,
                rtc_nominal_tick_hz=None,
                accepted_packet_count=None,
                first_imu_timestamp_raw=None,
                last_imu_timestamp_raw=None,
                first_rtc_tick=None,
                last_rtc_tick=None,
                rtc_wrap_ticks=None,
                first_unix_time_ms=None,
                last_unix_time_ms=None,
                first_unix_time_us=None,
                last_unix_time_us=None,
                raw_hex=raw_hex,
            )
    return OnlineEndPayload(
        session_id=session_id,
        stop_reason=stop_reason,
        produced_raw=produced_raw,
        produced_summary=produced_summary,
        produced_event=produced_event,
        acked_raw=acked_raw,
        acked_summary=acked_summary,
        acked_event=acked_event,
        pending_high_water=pending_high_water,
        clock_meta=clock_meta,
        stress=stress,
    )


def parse_online_record_fragment(frame: ExportFrame) -> OnlineRecordFragment:
    if frame.frame_type != ONLINE_FRAME_RECORD:
        raise ValueError(f"not an ONLINE_RECORD frame: 0x{frame.frame_type:02X}")
    payload = bytes(frame.payload)
    if len(payload) < ONLINE_RECORD_FRAGMENT_HEADER_BYTES:
        raise ValueError(f"ONLINE_RECORD payload too short: {len(payload)}")
    (
        session_id,
        record_type,
        _reserved0,
        flags,
        record_id,
        offset,
        record_bytes,
        fragment_len,
        _reserved1,
        crc32,
    ) = struct.unpack("<IBBHIIIHHI", payload[:ONLINE_RECORD_FRAGMENT_HEADER_BYTES])
    if len(payload) != ONLINE_RECORD_FRAGMENT_HEADER_BYTES + fragment_len:
        raise ValueError(
            f"ONLINE_RECORD fragment length mismatch: {fragment_len} != "
            f"{len(payload) - ONLINE_RECORD_FRAGMENT_HEADER_BYTES}"
        )
    if record_type not in (ONLINE_RECORD_RAW, ONLINE_RECORD_SUMMARY, ONLINE_RECORD_EVENT, RECORD_STRESS):
        raise ValueError(f"ONLINE_RECORD invalid type: {record_type}")
    if record_type == RECORD_STRESS and record_bytes > 4352:
        raise ValueError("STRESS record exceeds maximum length")
    if record_bytes < ONLINE_RECORD_HEADER_BYTES:
        raise ValueError(f"ONLINE_RECORD record_bytes too small: {record_bytes}")
    if offset + fragment_len > record_bytes:
        raise ValueError(
            f"ONLINE_RECORD fragment exceeds record size: offset={offset} len={fragment_len} total={record_bytes}"
        )
    if (flags & EXPORT_FLAG_LAST_FRAME) and (offset + fragment_len != record_bytes):
        raise ValueError(
            "ONLINE_RECORD last fragment does not end at record_bytes: "
            f"offset={offset} len={fragment_len} total={record_bytes}"
        )
    return OnlineRecordFragment(
        session_id=session_id,
        record_type=record_type,
        flags=flags,
        record_id=record_id,
        offset=offset,
        record_bytes=record_bytes,
        fragment_len=fragment_len,
        crc32=crc32,
        data=payload[ONLINE_RECORD_FRAGMENT_HEADER_BYTES:],
        chunk_seq=frame.chunk_seq,
    )


def parse_online_spool_header(record_data: bytes | bytearray) -> OnlineSpoolHeader:
    data = bytes(record_data)
    if len(data) < ONLINE_RECORD_HEADER_BYTES:
        raise ValueError(f"online record too short for spool header: {len(data)}")
    magic, version, header_len, session_id = struct.unpack("<IHHI", data[:12])
    record_type = data[12]
    flags = data[13]
    record_id, payload_bytes, source_record_bytes, crc32 = struct.unpack("<IIII", data[16:32])
    extension_version: int | None = None
    capture_segment_id: int | None = None
    segment_start_offset_ms: int | None = None
    source_id: int | None = None
    # Preserve the primary magic/version/length error for erased or malformed pages.
    # Extension fields are meaningful only after the fixed header is recognizable.
    if (
        magic == ONLINE_RECORD_MAGIC
        and version == ONLINE_RECORD_VERSION
        and header_len == ONLINE_RECORD_HEADER_BYTES
        and (flags & ONLINE_SPOOL_HEADER_FLAG_SEGMENT_META) != 0
    ):
        extension_version = struct.unpack_from("<H", data, 14)[0]
        if extension_version != ONLINE_SPOOL_HEADER_EXTENSION_VERSION:
            raise ValueError(
                "ONLINE_RECORD unsupported segment extension version: "
                f"{extension_version}"
            )
        capture_segment_id, segment_start_offset_ms, source_id = struct.unpack_from(
            "<III", data, 32
        )
        if capture_segment_id == 0:
            raise ValueError("ONLINE_RECORD segment metadata has zero capture_segment_id")
    return OnlineSpoolHeader(
        magic=magic,
        version=version,
        header_len=header_len,
        session_id=session_id,
        record_type=record_type,
        flags=flags,
        record_id=record_id,
        payload_bytes=payload_bytes,
        source_record_bytes=source_record_bytes,
        crc32=crc32,
        extension_version=extension_version,
        capture_segment_id=capture_segment_id,
        segment_start_offset_ms=segment_start_offset_ms,
        source_id=source_id,
    )


def _parse_online_continuous_raw_v2(
    data: bytes,
    header: OnlineSpoolHeader,
) -> OnlineContinuousRawInfo:
    source = data[header.header_len:]
    _magic, _version, raw_header_bytes = struct.unpack_from("<IHH", source, 0)
    if raw_header_bytes != FE_RAW_HEADER_BYTES:
        raise ValueError(f"continuous RAW header_bytes error: {raw_header_bytes}")
    if len(source) != FE_RAW_CONTINUOUS_SOURCE_RECORD_BYTES:
        raise ValueError(
            f"continuous RAW source bytes error: {len(source)}"
        )

    start_seq, end_seq, _hf_frames, packet_count = struct.unpack_from("<IIII", source, 36)
    first_timestamp_raw, last_timestamp_raw = struct.unpack_from("<HH", source, 52)
    section_count, payload_offset, payload_bytes, bucket_bytes = struct.unpack_from(
        "<IIII", source, 56
    )
    page_count, status = struct.unpack_from("<II", source, 84)
    flags = struct.unpack_from("<I", source, 72)[0]
    if (flags & FE_RAW_FLAG_CONTINUOUS_800HZ) == 0:
        raise ValueError("continuous RAW flag missing")
    if section_count != 1:
        raise ValueError(f"continuous RAW section_count error: {section_count}")
    if payload_offset != FE_RAW_HEADER_PAGE_BYTES:
        raise ValueError(f"continuous RAW payload_offset error: {payload_offset}")
    if bucket_bytes != FE_RAW_CONTINUOUS_ONLINE_RECORD_BYTES:
        raise ValueError(f"continuous RAW bucket_bytes error: {bucket_bytes}")
    if page_count != FE_RAW_CONTINUOUS_SOURCE_RECORD_BYTES // FE_RAW_HEADER_PAGE_BYTES:
        raise ValueError(f"continuous RAW page_count error: {page_count}")
    if status != FE_RAW_STATUS_COMMITTED:
        raise ValueError(f"continuous RAW status error: {status}")
    if packet_count < 1 or packet_count > FE_RAW_CONTINUOUS_PACKETS_PER_RECORD:
        raise ValueError(f"continuous RAW packet_count error: {packet_count}")
    if end_seq != ((start_seq + packet_count - 1) & 0xFFFFFFFF):
        raise ValueError(
            f"continuous RAW sequence span error: start={start_seq} end={end_seq} count={packet_count}"
        )

    section_offset = payload_offset
    if section_offset + FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES > len(source):
        raise ValueError("continuous RAW section header truncated")
    (
        section_type,
        section_header_bytes,
        sample_hz,
        packet_bytes,
        section_packet_count,
        section_first_timestamp,
        section_last_timestamp,
        section_payload_bytes,
        _section_flags,
    ) = struct.unpack_from("<HHIIIHHII", source, section_offset)
    if section_type != FE_RAW_CONTINUOUS_SECTION_TYPE:
        raise ValueError(f"continuous RAW section type error: {section_type}")
    if section_header_bytes != FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES:
        raise ValueError(
            f"continuous RAW section header bytes error: {section_header_bytes}"
        )
    if sample_hz != FE_RAW_CONTINUOUS_SAMPLE_HZ:
        raise ValueError(f"continuous RAW sample_hz error: {sample_hz}")
    if packet_bytes != FE_RAW_CONTINUOUS_PACKET_BYTES:
        raise ValueError(f"continuous RAW packet_bytes error: {packet_bytes}")
    if section_packet_count != packet_count:
        raise ValueError(
            f"continuous RAW section count mismatch: {section_packet_count} != {packet_count}"
        )
    expected_packet_payload = packet_count * FE_RAW_CONTINUOUS_PACKET_BYTES
    if section_payload_bytes != expected_packet_payload:
        raise ValueError(
            f"continuous RAW packet payload mismatch: {section_payload_bytes} != {expected_packet_payload}"
        )
    expected_payload_bytes = FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES + expected_packet_payload
    if payload_bytes != expected_payload_bytes or header.payload_bytes != expected_payload_bytes:
        raise ValueError(
            "continuous RAW payload length mismatch: "
            f"raw={payload_bytes} outer={header.payload_bytes} expected={expected_payload_bytes}"
        )

    packet_data_offset = header.header_len + payload_offset + section_header_bytes
    packet_end = packet_data_offset + expected_packet_payload
    if packet_end > len(data):
        raise ValueError("continuous RAW packet data truncated")
    first_packet = data[packet_data_offset:packet_data_offset + packet_bytes]
    last_packet_offset = packet_end - packet_bytes
    last_packet = data[last_packet_offset:packet_end]
    actual_first_timestamp = int.from_bytes(first_packet[14:16], "big")
    actual_last_timestamp = int.from_bytes(last_packet[14:16], "big")
    if not (
        first_timestamp_raw
        == section_first_timestamp
        == actual_first_timestamp
    ):
        raise ValueError("continuous RAW first timestamp binding mismatch")
    if not (
        last_timestamp_raw
        == section_last_timestamp
        == actual_last_timestamp
    ):
        raise ValueError("continuous RAW last timestamp binding mismatch")

    return OnlineContinuousRawInfo(
        sample_hz=sample_hz,
        packet_bytes=packet_bytes,
        packet_count=packet_count,
        start_seq=start_seq,
        end_seq=end_seq,
        first_timestamp_raw=actual_first_timestamp,
        last_timestamp_raw=actual_last_timestamp,
        packet_data_offset=packet_data_offset,
    )


def _parse_online_raw_v3(
    data: bytes,
    header: OnlineSpoolHeader,
) -> tuple[OnlineContinuousRawInfo | None, OnlineMagRawInfo | None]:
    source = data[header.header_len:]
    _magic, _version, raw_header_bytes = struct.unpack_from("<IHH", source, 0)
    if raw_header_bytes != FE_RAW_HEADER_BYTES:
        raise ValueError(f"RAW v3 header_bytes error: {raw_header_bytes}")
    if len(source) % FE_RAW_HEADER_PAGE_BYTES != 0:
        raise ValueError(f"RAW v3 source is not page aligned: {len(source)}")

    start_seq, end_seq, _hf_frames, packet_count = struct.unpack_from("<IIII", source, 36)
    first_timestamp_raw, last_timestamp_raw = struct.unpack_from("<HH", source, 52)
    section_count, payload_offset, payload_bytes, bucket_bytes = struct.unpack_from(
        "<IIII", source, 56
    )
    flags = struct.unpack_from("<I", source, 72)[0]
    page_count, status = struct.unpack_from("<II", source, 84)
    if section_count not in (1, 2):
        raise ValueError(f"RAW v3 section_count error: {section_count}")
    if payload_offset != FE_RAW_HEADER_PAGE_BYTES:
        raise ValueError(f"RAW v3 payload_offset error: {payload_offset}")
    if payload_offset + payload_bytes > len(source):
        raise ValueError("RAW v3 logical payload is truncated")
    if header.payload_bytes != payload_bytes:
        raise ValueError(
            f"RAW v3 outer payload mismatch: {header.payload_bytes} != {payload_bytes}"
        )
    if bucket_bytes < len(data) or bucket_bytes % FE_RAW_HEADER_PAGE_BYTES != 0:
        raise ValueError(f"RAW v3 bucket_bytes error: {bucket_bytes}")
    if page_count != len(source) // FE_RAW_HEADER_PAGE_BYTES:
        raise ValueError(f"RAW v3 page_count error: {page_count}")
    if status != FE_RAW_STATUS_COMMITTED:
        raise ValueError(f"RAW v3 status error: {status}")

    offset = payload_offset
    logical_end = payload_offset + payload_bytes
    continuous: OnlineContinuousRawInfo | None = None
    mag: OnlineMagRawInfo | None = None

    for _section_index in range(section_count):
        if offset + FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES > logical_end:
            raise ValueError("RAW v3 section header truncated")
        (
            section_type,
            section_header_bytes,
            sample_hz,
            item_bytes,
            item_count,
            section_first_timestamp,
            section_last_timestamp,
            section_payload_bytes,
            section_flags,
        ) = struct.unpack_from("<HHIIIHHII", source, offset)
        if section_header_bytes < FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES:
            raise ValueError(f"RAW v3 section header too short: {section_header_bytes}")
        data_offset = offset + section_header_bytes
        data_end = data_offset + section_payload_bytes
        if data_end > logical_end:
            raise ValueError("RAW v3 section payload truncated")

        if section_type == FE_RAW_CONTINUOUS_SECTION_TYPE:
            if continuous is not None:
                raise ValueError("RAW v3 duplicate IMU section")
            if section_header_bytes != FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES:
                raise ValueError(f"RAW v3 IMU header bytes error: {section_header_bytes}")
            if sample_hz != FE_RAW_CONTINUOUS_SAMPLE_HZ:
                raise ValueError(f"RAW v3 IMU sample_hz error: {sample_hz}")
            if item_bytes != FE_RAW_CONTINUOUS_PACKET_BYTES:
                raise ValueError(f"RAW v3 IMU packet bytes error: {item_bytes}")
            if item_count < 1 or item_count > FE_RAW_CONTINUOUS_PACKETS_PER_RECORD:
                raise ValueError(f"RAW v3 IMU packet count error: {item_count}")
            if section_payload_bytes != item_count * item_bytes:
                raise ValueError("RAW v3 IMU payload length mismatch")
            first_packet = source[data_offset:data_offset + item_bytes]
            last_packet = source[data_end - item_bytes:data_end]
            actual_first_timestamp = int.from_bytes(first_packet[14:16], "big")
            actual_last_timestamp = int.from_bytes(last_packet[14:16], "big")
            if not (
                first_timestamp_raw
                == section_first_timestamp
                == actual_first_timestamp
            ):
                raise ValueError("RAW v3 IMU first timestamp binding mismatch")
            if not (
                last_timestamp_raw
                == section_last_timestamp
                == actual_last_timestamp
            ):
                raise ValueError("RAW v3 IMU last timestamp binding mismatch")
            continuous = OnlineContinuousRawInfo(
                sample_hz=sample_hz,
                packet_bytes=item_bytes,
                packet_count=item_count,
                start_seq=start_seq,
                end_seq=end_seq,
                first_timestamp_raw=actual_first_timestamp,
                last_timestamp_raw=actual_last_timestamp,
                packet_data_offset=header.header_len + data_offset,
            )
        elif section_type == FE_RAW_MAG_SECTION_TYPE:
            if mag is not None:
                raise ValueError("RAW v3 duplicate MAG section")
            if section_header_bytes != FE_RAW_MAG_SECTION_HEADER_BYTES:
                raise ValueError(f"RAW v3 MAG header bytes error: {section_header_bytes}")
            if sample_hz != FE_RAW_MAG_SAMPLE_HZ:
                raise ValueError(f"RAW v3 MAG sample_hz error: {sample_hz}")
            if item_bytes != FE_RAW_MAG_SAMPLE_BYTES:
                raise ValueError(f"RAW v3 MAG sample bytes error: {item_bytes}")
            if item_count < 1 or item_count > FE_RAW_MAG_MAX_SAMPLES_PER_RECORD:
                raise ValueError(f"RAW v3 MAG sample count error: {item_count}")
            if section_first_timestamp != 0 or section_last_timestamp != 0:
                raise ValueError("RAW v3 MAG legacy timestamps must be zero")
            if section_payload_bytes != item_count * item_bytes:
                raise ValueError("RAW v3 MAG payload length mismatch")
            if (section_flags & FE_RAW_MAG_SECTION_FLAGS_REQUIRED) != FE_RAW_MAG_SECTION_FLAGS_REQUIRED:
                raise ValueError(f"RAW v3 MAG section flags error: 0x{section_flags:08X}")
            read_error_count, missed_deadline_count = struct.unpack_from("<II", source, offset + 28)
            samples: list[OnlineMagRawSample] = []
            previous_elapsed: int | None = None
            for sample_index in range(item_count):
                sample_offset = data_offset + sample_index * item_bytes
                elapsed_us = struct.unpack_from("<I", source, sample_offset)[0]
                if previous_elapsed is not None:
                    elapsed_delta = (elapsed_us - previous_elapsed) & 0xFFFFFFFF
                    if elapsed_delta == 0 or elapsed_delta >= (1 << 31):
                        raise ValueError(
                            "RAW v3 MAG elapsed_us is not strictly increasing modulo 2^32: "
                            f"current={elapsed_us} previous={previous_elapsed}"
                        )
                raw9 = bytes(source[sample_offset + 4:sample_offset + 13])
                if len(raw9) != FE_RAW_MAG_RAW_BYTES:
                    raise ValueError("RAW v3 MAG raw9 truncated")
                samples.append(OnlineMagRawSample(elapsed_us=elapsed_us, raw9=raw9))
                previous_elapsed = elapsed_us
            mag = OnlineMagRawInfo(
                sample_hz=sample_hz,
                sample_bytes=item_bytes,
                sample_count=item_count,
                section_flags=section_flags,
                read_error_count=read_error_count,
                missed_deadline_count=missed_deadline_count,
                sample_data_offset=header.header_len + data_offset,
                first_elapsed_us=samples[0].elapsed_us,
                last_elapsed_us=samples[-1].elapsed_us,
                samples=tuple(samples),
            )
        else:
            raise ValueError(f"RAW v3 unsupported section type: {section_type}")
        offset = data_end

    if offset != logical_end:
        raise ValueError(f"RAW v3 section bytes mismatch: {offset} != {logical_end}")
    # Stop tails may contain IMU only or MAG only. Source flags describe
    # the sections actually present, not the session's sensor capabilities.
    if bool(flags & FE_RAW_FLAG_MAG_RAW_100HZ) != (mag is not None):
        raise ValueError("RAW v3 MAG section/source flag mismatch")
    if bool(flags & FE_RAW_FLAG_CONTINUOUS_800HZ) != (continuous is not None):
        raise ValueError("RAW v3 IMU section/source flag mismatch")
    if continuous is None:
        if section_count != 1 or packet_count != 0:
            raise ValueError("RAW v3 MAG-only tail has invalid IMU metadata")
    else:
        if packet_count != continuous.packet_count:
            raise ValueError("RAW v3 source/section IMU count mismatch")
        if end_seq != ((start_seq + packet_count - 1) & 0xFFFFFFFF):
            raise ValueError("RAW v3 IMU sequence span mismatch")

    padding = source[logical_end:]
    if any(value != 0xFF for value in padding):
        raise ValueError("RAW v3 padding is not erased 0xFF")
    return continuous, mag


def parse_online_raw_sections(
    record_data: bytes | bytearray,
    header: OnlineSpoolHeader,
) -> tuple[int | None, OnlineContinuousRawInfo | None, OnlineMagRawInfo | None]:
    data = bytes(record_data)
    if header.record_type != ONLINE_RECORD_RAW:
        return None, None, None
    source = data[header.header_len:]
    if len(source) < FE_RAW_HEADER_BYTES:
        return None, None, None
    magic, version, _raw_header_bytes = struct.unpack_from("<IHH", source, 0)
    if magic != FE_RAW_MAGIC:
        return None, None, None
    if version == FE_RAW_CONTINUOUS_VERSION:
        return version, _parse_online_continuous_raw_v2(data, header), None
    if version == FE_RAW_IMU_MAG_VERSION:
        continuous, mag = _parse_online_raw_v3(data, header)
        return version, continuous, mag
    raise ValueError(f"unsupported online RAW source version: {version}")


def parse_online_continuous_raw(
    record_data: bytes | bytearray,
    header: OnlineSpoolHeader,
) -> OnlineContinuousRawInfo | None:
    """Compatibility wrapper retained for callers that only need IMU metadata."""
    _version, continuous, _mag = parse_online_raw_sections(record_data, header)
    return continuous


def validate_online_completed_record(
    last_fragment: OnlineRecordFragment,
    record_data: bytes | bytearray,
) -> OnlineCompletedRecord:
    data = bytes(record_data)
    if len(data) != last_fragment.record_bytes:
        raise ValueError(
            f"ONLINE_RECORD assembled length mismatch: {len(data)} != {last_fragment.record_bytes}"
        )
    header = parse_online_spool_header(data)
    if header.magic != ONLINE_RECORD_MAGIC:
        raise ValueError(f"ONLINE_RECORD magic error: 0x{header.magic:08X}")
    if header.version != ONLINE_RECORD_VERSION:
        raise ValueError(f"ONLINE_RECORD version error: {header.version}")
    if header.header_len != ONLINE_RECORD_HEADER_BYTES:
        raise ValueError(f"ONLINE_RECORD header_len error: {header.header_len}")
    if header.session_id != last_fragment.session_id:
        raise ValueError(
            f"ONLINE_RECORD header session mismatch: {header.session_id} != {last_fragment.session_id}"
        )
    if header.record_type != last_fragment.record_type:
        raise ValueError(
            f"ONLINE_RECORD header type mismatch: {header.record_type} != {last_fragment.record_type}"
        )
    if header.record_id != last_fragment.record_id:
        raise ValueError(
            f"ONLINE_RECORD header id mismatch: {header.record_id} != {last_fragment.record_id}"
        )
    if header.source_record_bytes + header.header_len != len(data):
        raise ValueError(
            f"ONLINE_RECORD source size mismatch: {header.source_record_bytes}+{header.header_len} != {len(data)}"
        )
    if header.crc32 != last_fragment.crc32:
        raise ValueError(f"ONLINE_RECORD crc header/fragment mismatch: 0x{header.crc32:08X} != 0x{last_fragment.crc32:08X}")
    actual_crc = binascii.crc32(data[ONLINE_RECORD_HEADER_BYTES:]) & 0xFFFFFFFF
    if actual_crc != header.crc32:
        raise ValueError(f"ONLINE_RECORD crc mismatch: got=0x{header.crc32:08X} expected=0x{actual_crc:08X}")
    if header.record_type not in (ONLINE_RECORD_RAW, ONLINE_RECORD_SUMMARY, ONLINE_RECORD_EVENT, RECORD_STRESS):
        raise ValueError("ONLINE_RECORD invalid header type")
    stress = parse_stress_source(data[256:], header.session_id) if header.record_type == RECORD_STRESS else None
    if stress is not None and stress.data_bytes != header.payload_bytes:
        raise ValueError("STRESS payload byte count mismatch")
    raw_source_version, continuous_raw, mag_raw = parse_online_raw_sections(data, header)
    return OnlineCompletedRecord(
        session_id=last_fragment.session_id,
        record_type=last_fragment.record_type,
        record_id=last_fragment.record_id,
        record_bytes=len(data),
        payload_bytes=header.payload_bytes,
        source_record_bytes=header.source_record_bytes,
        crc32=header.crc32,
        data=data,
        header=header,
        raw_source_version=raw_source_version,
        continuous_raw=continuous_raw,
        mag_raw=mag_raw,
        stress=stress,
    )


def parse_feuf_export_end_payload(payload: bytes | bytearray) -> FEUFExportEnd:
    data = bytes(payload)
    if len(data) != 20:
        raise ValueError(f"FEUF EXPORT_END payload length error: {len(data)}")
    session_id, frames_sent, payload_bytes_sent, section_crc32_xor, status = struct.unpack("<IIIII", data)
    return FEUFExportEnd(
        session_id=session_id,
        frames_sent=frames_sent,
        payload_bytes_sent=payload_bytes_sent,
        section_crc32_xor=section_crc32_xor,
        status=status,
    )


def parse_feuf_stream(stream: bytes | bytearray) -> FEUFParseResult:
    data = bytes(stream)
    pos = 0
    frames: list[FEUFFrame] = []
    sections: dict[int, bytearray] = {}
    manifest: FEUFManifest | None = None
    export_end: FEUFExportEnd | None = None

    while pos < len(data):
        if pos + FEUF_HEADER_BYTES > len(data):
            raise ValueError(f"truncated FEUF header at offset {pos}")
        header = data[pos:pos + FEUF_HEADER_BYTES]
        (
            magic,
            version,
            header_bytes,
            frame_type,
            flags,
            seq,
            section_id,
            section_offset,
            payload_len,
            _reserved0,
            payload_crc32,
            header_crc32,
        ) = struct.unpack("<IHHHHIIIHHII", header)
        if magic != FEUF_MAGIC:
            raise ValueError(f"bad FEUF magic at offset {pos}: 0x{magic:08X}")
        if version != FEUF_VERSION:
            raise ValueError(f"bad FEUF version at offset {pos}: {version}")
        if header_bytes != FEUF_HEADER_BYTES:
            raise ValueError(f"bad FEUF header size at offset {pos}: {header_bytes}")
        if frame_type == FEUF_FRAME_DATA and payload_len > FEUF_PROTOCOL_MAX_PAYLOAD_CAPACITY:
            raise ValueError(
                "FEUF DATA payload_len exceeds protocol max "
                f"seq={seq}: {payload_len} > {FEUF_PROTOCOL_MAX_PAYLOAD_CAPACITY}"
            )

        header_for_crc = bytearray(header)
        header_for_crc[32:36] = b"\x00\x00\x00\x00"
        actual_header_crc = binascii.crc32(header_for_crc) & 0xFFFFFFFF
        if actual_header_crc != header_crc32:
            raise ValueError(
                f"FEUF header CRC mismatch seq={seq}: got=0x{header_crc32:08X} expected=0x{actual_header_crc:08X}"
            )

        payload_start = pos + header_bytes
        payload_end = payload_start + payload_len
        if payload_end > len(data):
            raise ValueError(f"truncated FEUF payload seq={seq}")
        frame_payload = data[payload_start:payload_end]
        actual_payload_crc = binascii.crc32(frame_payload) & 0xFFFFFFFF if frame_payload else 0
        if actual_payload_crc != payload_crc32:
            raise ValueError(
                f"FEUF payload CRC mismatch seq={seq}: got=0x{payload_crc32:08X} expected=0x{actual_payload_crc:08X}"
            )

        frame = FEUFFrame(
            frame_type=frame_type,
            flags=flags,
            seq=seq,
            section_id=section_id,
            section_offset=section_offset,
            payload=frame_payload,
            payload_crc32=payload_crc32,
            header_crc32=header_crc32,
        )
        frames.append(frame)

        if frame_type == FEUF_FRAME_MANIFEST and section_id == FEUF_SECTION_MANIFEST:
            manifest = parse_feuf_manifest(frame_payload)
        elif frame_type == FEUF_FRAME_DATA:
            if manifest is not None and payload_len > manifest.payload_max:
                raise ValueError(
                    "FEUF DATA payload_len exceeds manifest payload_max "
                    f"seq={seq}: {payload_len} > {manifest.payload_max}"
                )
            section_buf = sections.setdefault(section_id, bytearray())
            if section_offset != len(section_buf):
                raise ValueError(
                    f"FEUF section offset mismatch section={section_id} got={section_offset} expected={len(section_buf)}"
                )
            section_buf.extend(frame_payload)
        elif frame_type == FEUF_FRAME_EXPORT_END:
            export_end = parse_feuf_export_end_payload(frame_payload)

        pos = payload_end

    if manifest is not None:
        for frame in frames:
            if frame.frame_type == FEUF_FRAME_DATA and len(frame.payload) > manifest.payload_max:
                raise ValueError(
                    "FEUF DATA payload_len exceeds manifest payload_max "
                    f"seq={frame.seq}: {len(frame.payload)} > {manifest.payload_max}"
                )

    parsed_sections = {section_id: bytes(payload) for section_id, payload in sections.items()}
    training_begin = _parse_optional_training_record(parsed_sections.get(FEUF_SECTION_TRAINING_BEGIN), "TRNB")
    training_end = _parse_optional_training_record(parsed_sections.get(FEUF_SECTION_TRAINING_END), "TRNE")
    return FEUFParseResult(
        frames=tuple(frames),
        manifest=manifest,
        training_begin=training_begin,
        training_end=training_end,
        export_end=export_end,
        sections=parsed_sections,
    )


def parse_feuf_manifest(payload: bytes | bytearray) -> FEUFManifest:
    data = bytes(payload)
    if len(data) < 4:
        raise ValueError(f"FEUF manifest length error: {len(data)}")
    protocol_version = struct.unpack_from("<I", data, 0)[0]
    if protocol_version == 2:
        return _parse_feuf_manifest_v2(data)
    if protocol_version == 3:
        return _parse_feuf_manifest_v3(data)
    raise ValueError(f"unsupported FEUF manifest protocol_version: {protocol_version}")


def _validate_feuf_payload_max(payload_max: int) -> int:
    if payload_max <= 0:
        raise ValueError(f"FEUF manifest payload_max invalid: {payload_max}")
    if payload_max > FEUF_PROTOCOL_MAX_PAYLOAD_CAPACITY:
        raise ValueError(
            "FEUF manifest payload_max exceeds protocol max "
            f"{payload_max} > {FEUF_PROTOCOL_MAX_PAYLOAD_CAPACITY}"
        )
    return payload_max


def _parse_feuf_manifest_v2(data: bytes) -> FEUFManifest:
    if len(data) != 108:
        raise ValueError(f"FEUF manifest length error: {len(data)}")
    data_for_crc = bytearray(data)
    data_for_crc[84:88] = b"\x00\x00\x00\x00"
    actual_crc = binascii.crc32(data_for_crc) & 0xFFFFFFFF
    saved_crc = struct.unpack_from("<I", data, 84)[0]
    if actual_crc != saved_crc:
        raise ValueError(f"FEUF manifest CRC mismatch: got=0x{saved_crc:08X} expected=0x{actual_crc:08X}")

    fields = struct.unpack("<27I", data)
    return FEUFManifest(
        protocol_version=fields[0],
        session_version=fields[1],
        round=fields[2],
        stop_reason=fields[3],
        live_hz=fields[4],
        ois_hz=fields[5],
        raw_count=fields[6],
        summary_count=fields[7],
        event_count=fields[8],
        meta_bytes=fields[9],
        raw_bytes=fields[10],
        summary_bytes=fields[11],
        event_bytes=fields[12],
        payload_max=_validate_feuf_payload_max(fields[17]),
        total_sections=fields[18],
        total_frames_est=fields[19],
        total_payload_bytes=fields[20],
        manifest_crc32=fields[21],
        training_section_count=fields[22],
        training_begin_bytes=fields[23],
        training_end_bytes=fields[24],
        training_flags=fields[25],
        section_mask_all=fields[26],
        manifest_bytes=108,
        header_bytes=108,
    )


def _parse_feuf_manifest_v3(data: bytes) -> FEUFManifest:
    if len(data) < 8:
        raise ValueError(f"FEUF manifest v3 length error: {len(data)}")
    manifest_bytes = struct.unpack_from("<I", data, 4)[0]
    if manifest_bytes < FEUF_MANIFEST_V3_MIN_BYTES:
        raise ValueError(f"FEUF manifest v3 manifest_bytes too small: {manifest_bytes}")
    if manifest_bytes > len(data):
        raise ValueError(f"FEUF manifest v3 manifest_bytes exceeds payload: {manifest_bytes} > {len(data)}")
    if len(data) != manifest_bytes:
        raise ValueError(f"FEUF manifest length error: {len(data)} != {manifest_bytes}")

    data_for_crc = bytearray(data[:manifest_bytes])
    data_for_crc[FEUF_MANIFEST_V3_CRC_OFFSET:FEUF_MANIFEST_V3_CRC_OFFSET + 4] = b"\x00\x00\x00\x00"
    actual_crc = binascii.crc32(data_for_crc) & 0xFFFFFFFF
    saved_crc = struct.unpack_from("<I", data, FEUF_MANIFEST_V3_CRC_OFFSET)[0]
    if actual_crc != saved_crc:
        raise ValueError(f"FEUF manifest CRC mismatch: got=0x{saved_crc:08X} expected=0x{actual_crc:08X}")

    def u16(offset: int) -> int:
        return struct.unpack_from("<H", data, offset)[0]

    def u32(offset: int) -> int:
        return struct.unpack_from("<I", data, offset)[0]

    def u64(offset: int) -> int:
        return struct.unpack_from("<Q", data, offset)[0]

    header_bytes = u16(8)
    session_version = u16(10)
    return FEUFManifest(
        protocol_version=3,
        session_version=session_version,
        round=u32(36),
        stop_reason=u32(56),
        live_hz=u32(60),
        ois_hz=u32(64),
        raw_count=u32(76),
        summary_count=u32(96),
        event_count=u32(116),
        meta_bytes=u32(128),
        raw_bytes=u32(132),
        summary_bytes=u32(136),
        event_bytes=u32(140),
        payload_max=_validate_feuf_payload_max(u32(144)),
        total_sections=u32(148),
        total_frames_est=u32(152),
        total_payload_bytes=u32(156),
        manifest_crc32=saved_crc,
        training_section_count=u32(160),
        training_begin_bytes=u32(164),
        training_end_bytes=u32(168),
        training_flags=u32(172),
        section_mask_all=u32(176),
        manifest_bytes=manifest_bytes,
        header_bytes=header_bytes,
        session_uid=u32(12),
        export_index=u32(16),
        export_total=u32(20),
        user_id=u32(24),
        training_id=u32(28),
        session_seq=u32(32),
        start_time_ms=u64(40),
        end_time_ms=u64(48),
        raw_first_bucket=u32(68),
        raw_limit_bucket=u32(72),
        raw_used_bytes=u32(80),
        raw_base_addr=u32(68),
        raw_data_begin_addr=u32(68),
        raw_data_end_addr=u32(72),
        raw_reclaim_end_addr=u32(84),
        summary_base_addr=u32(88),
        summary_limit_addr=u32(92),
        summary_data_begin_addr=u32(88),
        summary_data_end_addr=u32(92),
        summary_reclaim_end_addr=u32(104),
        summary_used_bytes=u32(100),
        event_base_addr=u32(108),
        event_limit_addr=u32(112),
        event_data_begin_addr=u32(108),
        event_data_end_addr=u32(112),
        event_reclaim_end_addr=u32(124),
        event_used_bytes=u32(120),
        flags=u32(180),
        manifest_stream_crc32=u32(184),
    )


def parse_training_record(payload: bytes | bytearray) -> TrainingRecord:
    data = bytes(payload)
    if len(data) != 64:
        raise ValueError(f"training record length error: {len(data)}")
    saved_crc = struct.unpack_from("<I", data, 60)[0]
    actual_crc = binascii.crc32(data[:60]) & 0xFFFFFFFF
    if saved_crc != actual_crc:
        raise ValueError(f"training record CRC mismatch: got=0x{saved_crc:08X} expected=0x{actual_crc:08X}")
    magic = data[:4].decode("ascii", errors="strict")
    if magic not in {"TRNB", "TRNE"}:
        raise ValueError(f"bad training record magic: {magic}")

    version, record_type, header_len, flags = struct.unpack_from("<BBBB", data, 4)
    user_id, training_id, session_seq, capture_round = struct.unpack_from("<IIII", data, 8)
    start_time_ms = struct.unpack_from("<Q", data, 24)[0]
    end_time_ms = struct.unpack_from("<Q", data, 32)[0]
    time_source_start, time_source_end, source, stop_reason = struct.unpack_from("<BBBB", data, 40)
    data_flags, raw_block_count, summary_count, event_count = struct.unpack_from("<IIII", data, 44)

    return TrainingRecord(
        magic=magic,
        version=version,
        record_type=record_type,
        header_len=header_len,
        flags=flags,
        user_id=user_id,
        training_id=training_id,
        session_seq=session_seq,
        capture_round=capture_round,
        start_time_ms=start_time_ms,
        end_time_ms=end_time_ms,
        time_source_start=time_source_start,
        time_source_end=time_source_end,
        source=source,
        stop_reason=stop_reason,
        data_flags=data_flags,
        raw_block_count=raw_block_count,
        summary_count=summary_count,
        event_count=event_count,
        crc32=saved_crc,
    )


def _parse_optional_training_record(payload: bytes | None, expected_magic: str) -> TrainingRecord | None:
    if payload is None:
        return None
    record = parse_training_record(payload)
    if record.magic != expected_magic:
        raise ValueError(f"expected {expected_magic}, got {record.magic}")
    return record


def parse_zy100_ack(data: bytes | bytearray) -> ZY100Ack:
    payload = bytes(data)
    if len(payload) != ZY100_ACK_SIZE:
        raise ValueError(f"ACK length error: {len(payload)}")

    magic, version, cmd_echo, seq_echo, status, device_state, exec_mode, reserved, user_id, training_id, detail = (
        struct.unpack("<BBBBBBBBIII", payload)
    )

    if magic != 0x5A:
        raise ValueError(f"ACK magic error: 0x{magic:02X}")
    if version != 0x01:
        raise ValueError(f"ACK version error: 0x{version:02X}")

    return ZY100Ack(
        cmd_echo=cmd_echo,
        seq_echo=seq_echo,
        status=status,
        device_state=device_state,
        exec_mode=exec_mode,
        reserved=reserved,
        user_id=user_id,
        training_id=training_id,
        detail=detail,
        raw=payload,
    )


def parse_connection_state_snapshot(
    data: bytes | bytearray | ZY100Ack,
) -> ConnectionStateSnapshot:
    ack = data if isinstance(data, ZY100Ack) else parse_zy100_ack(data)
    if ack.cmd_echo != CMD_CONNECTION_STATE_NOTIFY:
        raise ValueError(f"not a connection-state snapshot: 0x{ack.cmd_echo:02X}")
    if ack.reserved not in {
        BOOTSTRAP_VERSION_V1,
        BOOTSTRAP_VERSION_V2,
        BOOTSTRAP_VERSION_V3,
        BOOTSTRAP_VERSION_V4,
    }:
        raise ValueError(f"unsupported bootstrap version: {ack.reserved}")
    ready_bit_count = {
        BOOTSTRAP_VERSION_V1: 12,
        BOOTSTRAP_VERSION_V2: 13,
        BOOTSTRAP_VERSION_V3: 14,
        BOOTSTRAP_VERSION_V4: 15,
    }[ack.reserved]
    known_mask = (1 << ready_bit_count) - 1
    if ack.training_id & ~known_mask:
        raise ValueError(f"unsupported connection readiness bits: 0x{ack.training_id:08X}")
    if ack.reserved == BOOTSTRAP_VERSION_V1 and ack.training_id & (
        CONNECTION_READY_OFFLINE_CAPTURE_OBSERVER
        | CONNECTION_READY_OFFLINE_SESSION_ATTENTION
    ):
        raise ValueError("bootstrap v1 reserved readiness bit is set")
    return ConnectionStateSnapshot(
        revision=ack.seq_echo,
        overall_status=ack.status,
        device_state=ack.device_state,
        suggested_action=ack.exec_mode,
        bootstrap_version=ack.reserved,
        generation=ack.user_id,
        ready_bits=ack.training_id,
        stage=ack.detail & 0xFF,
        reason=(ack.detail >> 8) & 0xFF,
        retry_ms=(ack.detail >> 16) & 0xFFFF,
        raw=ack.raw,
    )


def is_time_sync_ack_ok(
    ack: ZY100Ack,
    expected_seq: int,
    expected_user_id: int = 1,
    expected_training_id: int = 1,
) -> bool:
    if not 1 <= expected_user_id <= 0xFFFFFFFF:
        return False
    if expected_training_id <= 0:
        expected_training_id = 1
    return (
        ack.cmd_echo == CMD_TIME_SYNC
        and ack.seq_echo == (expected_seq & 0xFF)
        and ack.status == 0x00
        and ack.exec_mode == 0x02
        and ack.user_id == (expected_user_id & 0xFFFFFFFF)
        and ack.training_id == (expected_training_id & 0xFFFFFFFF)
    )
