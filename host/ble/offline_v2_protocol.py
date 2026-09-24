from __future__ import annotations

import binascii
import math
import struct
from dataclasses import dataclass
from typing import Final

from .zy100_protocol import (
    CMD_OFFLINE_CHUNK_ACK,
    CMD_OFFLINE_FINAL_CONFIRM,
    CMD_OFFLINE_RECLAIM_STATUS,
    CMD_OFFLINE_SESSION_BEGIN,
    CMD_OFFLINE_SESSION_LIST,
    CMD_OFFLINE_SESSION_RESUME,
    build_zy100_command,
)


OFFLINE_FRAME_SESSION_LIST: Final[int] = 0x21
OFFLINE_FRAME_SESSION_BEGIN: Final[int] = 0x22
OFFLINE_FRAME_SESSION_CHUNK: Final[int] = 0x23
OFFLINE_FRAME_SESSION_END: Final[int] = 0x24
OFFLINE_FRAME_RECLAIM_STATUS: Final[int] = 0x25
OFFLINE_FRAME_SYNC_ABORT: Final[int] = 0x26
OFFLINE_FRAME_TYPES: Final[frozenset[int]] = frozenset(
    {
        OFFLINE_FRAME_SESSION_LIST,
        OFFLINE_FRAME_SESSION_BEGIN,
        OFFLINE_FRAME_SESSION_CHUNK,
        OFFLINE_FRAME_SESSION_END,
        OFFLINE_FRAME_RECLAIM_STATUS,
        OFFLINE_FRAME_SYNC_ABORT,
    }
)

OFFLINE_OUTER_MAGIC: Final[int] = 0xE7
OFFLINE_OUTER_VERSION: Final[int] = 2
OFFLINE_OUTER_BYTES: Final[int] = 8
OFFLINE_FLAG_LAST: Final[int] = 0x01
OFFLINE_FLAG_RETRANSMIT: Final[int] = 0x02
OFFLINE_FLAG_LIST_LAST: Final[int] = 0x04

OFFLINE_LIST_HEADER_BYTES: Final[int] = 20
OFFLINE_LIST_ENTRY_BYTES: Final[int] = 40
OFFLINE_LIST_MAX_ENTRIES: Final[int] = 5
OFFLINE_MANIFEST_LEGACY_VERSION: Final[int] = 3
OFFLINE_MANIFEST_LEGACY_BYTES: Final[int] = 120
OFFLINE_MANIFEST_LEGACY_CRC_OFFSET: Final[int] = 116
OFFLINE_MANIFEST_VERSION: Final[int] = 4
OFFLINE_MANIFEST_BYTES: Final[int] = 124
OFFLINE_MANIFEST_OWNER_USER_ID_OFFSET: Final[int] = 116
OFFLINE_MANIFEST_CRC_OFFSET: Final[int] = 120
OFFLINE_MANIFEST_SUPPORTED_VERSIONS: Final[frozenset[int]] = frozenset(
    {OFFLINE_MANIFEST_LEGACY_VERSION, OFFLINE_MANIFEST_VERSION}
)
OFFLINE_CHUNK_HEADER_BYTES: Final[int] = 28
OFFLINE_CHUNK_DATA_MAX_BYTES: Final[int] = 200
OFFLINE_END_BYTES: Final[int] = 44
OFFLINE_RECLAIM_BYTES: Final[int] = 24
OFFLINE_ABORT_BYTES: Final[int] = 24

OFFLINE_EVENT_MAGIC: Final[int] = 0x3245464F
OFFLINE_EVENT_VERSION: Final[int] = 2
OFFLINE_EVENT_HEADER_BYTES: Final[int] = 48
OFFLINE_EVENT_RECORD_BYTES: Final[int] = 304
OFFLINE_EVENT_SLOT_BYTES: Final[int] = 512
OFFLINE_EVENT_CRC_OFFSET: Final[int] = 44
OFFLINE_EVENT_FLAGS_MASK: Final[int] = 0x001F
OFFLINE_EVENT_DIMENSIONS: Final[int] = 128
OFFLINE_Q_FRAC_BITS: Final[int] = 12
OFFLINE_SAMPLE_RATE_HZ: Final[int] = 800
OFFLINE_CONFIG_CRC32: Final[int] = 0x36657DE0
OFFLINE_CONTRACT_CRC32: Final[tuple[int, int, int, int, int]] = (
    0xDE63EA7B,
    0x791F19DD,
    0x38382FDC,
    0x6D7D2D01,
    0x04EA5EEB,
)
OFFLINE_SHOCK_ID_CRC32: Final[int] = 0xCA1B969B
OFFLINE_EVENT_VERSIONS: Final[tuple[int, int, int, int]] = (1, 1, 2, 2)

OFFLINE_LIST_FLAG_CLEAN: Final[int] = 0x01
OFFLINE_LIST_FLAG_ATTENTION: Final[int] = 0x02
OFFLINE_LIST_FLAG_RECOVERED: Final[int] = 0x04
OFFLINE_SESSION_HEALTH_NORMAL: Final[int] = 0
OFFLINE_SESSION_HEALTH_CONTROLLED_STOP: Final[int] = 1
OFFLINE_SESSION_HEALTH_ABNORMAL_FINALIZED: Final[int] = 2
OFFLINE_SESSION_HEALTH_RECOVERED_PREFIX: Final[int] = 3
OFFLINE_SESSION_HEALTH_NAMES: Final[dict[int, str]] = {
    OFFLINE_SESSION_HEALTH_NORMAL: "normal",
    OFFLINE_SESSION_HEALTH_CONTROLLED_STOP: "controlled_stop",
    OFFLINE_SESSION_HEALTH_ABNORMAL_FINALIZED: "abnormal_finalized",
    OFFLINE_SESSION_HEALTH_RECOVERED_PREFIX: "recovered_prefix",
}

RECLAIM_UNKNOWN: Final[int] = 0
RECLAIM_FINAL: Final[int] = 1
RECLAIM_CONFIRMED: Final[int] = 2
RECLAIM_RECLAIMING: Final[int] = 3
RECLAIM_TOMBSTONE: Final[int] = 4
RECLAIM_ERROR: Final[int] = 5
RECLAIM_PAUSED: Final[int] = 6
RECLAIM_STATE_NAMES: Final[dict[int, str]] = {
    RECLAIM_UNKNOWN: "unknown",
    RECLAIM_FINAL: "final",
    RECLAIM_CONFIRMED: "confirmed",
    RECLAIM_RECLAIMING: "reclaiming",
    RECLAIM_TOMBSTONE: "tombstone",
    RECLAIM_ERROR: "error",
    RECLAIM_PAUSED: "paused",
}


@dataclass(frozen=True, slots=True)
class OfflineOuterFrame:
    frame_type: int
    flags: int
    chunk_seq: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class OfflineSessionListEntry:
    session_id: int
    generation: int
    state: int
    flags: int
    format_version: int
    event_count: int
    logical_bytes: int
    stream_crc32: int
    start_unix_ms: int
    duration_ms: int
    config_crc32: int

    @property
    def finalized(self) -> bool:
        return self.state in {1, 2}

    @property
    def clean(self) -> bool:
        return bool(self.flags & OFFLINE_LIST_FLAG_CLEAN)

    @property
    def attention(self) -> bool:
        return bool(self.flags & OFFLINE_LIST_FLAG_ATTENTION)

    @property
    def recovered(self) -> bool:
        return bool(self.flags & OFFLINE_LIST_FLAG_RECOVERED)


@dataclass(frozen=True, slots=True)
class OfflineSessionListPage:
    generation: int
    cursor: int
    total: int
    next_cursor: int
    entries: tuple[OfflineSessionListEntry, ...]
    list_crc32: int

    @property
    def is_last(self) -> bool:
        return self.next_cursor == 0xFFFFFFFF


@dataclass(frozen=True, slots=True)
class OfflineSessionManifest:
    session_id: int
    transfer_id: int
    generation: int
    format_version: int
    event_record_bytes: int
    event_slot_bytes: int
    feature_dimensions: int
    q_frac_bits: int
    flags: int
    sample_rate_hz: int
    event_count: int
    logical_bytes: int
    stream_crc32: int
    start_unix_ms: int
    duration_ms: int
    contract_crc32: tuple[int, int, int, int, int]
    physical_span_bytes: int
    extent_bytes: int
    shock_id_crc32: int
    config_crc32: int
    manifest_crc32: int
    stop_reason: int = 0
    clean: bool = True
    health: int = OFFLINE_SESSION_HEALTH_NORMAL
    fifo_overflow_count: int = 0
    fifo_discard_count: int = 0
    time_gap_count: int = 0
    feature_drop_count: int = 0
    q12_clip_count: int = 0
    flash_error_count: int = 0
    owner_user_id: int | None = None

    @property
    def health_name(self) -> str:
        return OFFLINE_SESSION_HEALTH_NAMES.get(self.health, f"unknown_{self.health}")


@dataclass(frozen=True, slots=True)
class OfflineSessionChunk:
    session_id: int
    transfer_id: int
    offset: int
    total_bytes: int
    data_crc32: int
    prefix_crc32: int
    data: bytes

    @property
    def next_offset(self) -> int:
        return self.offset + len(self.data)


@dataclass(frozen=True, slots=True)
class OfflineSessionEnd:
    session_id: int
    transfer_id: int
    generation: int
    logical_bytes: int
    event_count: int
    stream_crc32: int
    manifest_crc32: int
    physical_span_bytes: int
    extent_bytes: int
    chunk_count: int


@dataclass(frozen=True, slots=True)
class OfflineReclaimStatus:
    session_id: int
    generation: int
    state: int
    erased_bytes: int
    extent_bytes: int

    @property
    def state_name(self) -> str:
        return RECLAIM_STATE_NAMES.get(self.state, f"unknown_{self.state}")


@dataclass(frozen=True, slots=True)
class OfflineSyncAbort:
    session_id: int
    transfer_id: int
    generation: int
    acked_offset: int
    reason: int
    detail: int


@dataclass(frozen=True, slots=True)
class OfflineEventRecord:
    flags: int
    session_id: int
    event_index: int
    center_sample_index: int
    peak_mag2_raw: int
    config_crc32: int
    shock_numerator: float
    shock_baseline: float
    shock_ratio_rounded_4dp: float
    detector_version: int
    shock_version: int
    feature_version: int
    codec_version: int
    record_crc32: int
    feature_q12: tuple[int, ...]

    @property
    def center_time_seconds(self) -> float:
        return self.center_sample_index / float(OFFLINE_SAMPLE_RATE_HZ)

    @property
    def shock_available(self) -> bool:
        return bool(self.flags & 0x0001)

    @property
    def valid_hit(self) -> bool:
        return bool(self.flags & 0x0002)

    @property
    def accel_clipped(self) -> bool:
        return bool(self.flags & 0x0004)

    @property
    def gyro_clipped(self) -> bool:
        return bool(self.flags & 0x0008)

    @property
    def q12_saturated(self) -> bool:
        return bool(self.flags & 0x0010)


def crc32_ieee(data: bytes | bytearray | memoryview, value: int = 0) -> int:
    return binascii.crc32(data, value) & 0xFFFFFFFF


def is_offline_v2_frame_type(frame_type: int) -> bool:
    return (frame_type & 0xFF) in OFFLINE_FRAME_TYPES


def parse_offline_outer_frame(data: bytes | bytearray) -> OfflineOuterFrame:
    raw = bytes(data)
    if len(raw) < OFFLINE_OUTER_BYTES:
        raise ValueError(f"OFFLINE V2 outer frame too short: {len(raw)}")
    magic, version, frame_type, flags, chunk_seq, payload_len = struct.unpack(
        "<BBBBHH", raw[:OFFLINE_OUTER_BYTES]
    )
    if magic != OFFLINE_OUTER_MAGIC:
        raise ValueError(f"OFFLINE V2 magic error: 0x{magic:02X}")
    if version != OFFLINE_OUTER_VERSION:
        raise ValueError(f"OFFLINE V2 version error: {version}")
    if payload_len != len(raw) - OFFLINE_OUTER_BYTES:
        raise ValueError(
            f"OFFLINE V2 payload length mismatch: {payload_len} != {len(raw) - OFFLINE_OUTER_BYTES}"
        )
    if not is_offline_v2_frame_type(frame_type):
        raise ValueError(f"not an OFFLINE V2 frame: 0x{frame_type:02X}")
    if flags & ~(OFFLINE_FLAG_LAST | OFFLINE_FLAG_RETRANSMIT | OFFLINE_FLAG_LIST_LAST):
        raise ValueError(f"OFFLINE V2 unsupported flags: 0x{flags:02X}")
    return OfflineOuterFrame(frame_type, flags, chunk_seq, raw[OFFLINE_OUTER_BYTES:])


def parse_session_list(frame: OfflineOuterFrame) -> OfflineSessionListPage:
    if frame.frame_type != OFFLINE_FRAME_SESSION_LIST:
        raise ValueError("not an OFFLINE SESSION_LIST frame")
    if frame.flags not in {0, OFFLINE_FLAG_LIST_LAST}:
        raise ValueError(f"OFFLINE SESSION_LIST invalid flags: 0x{frame.flags:02X}")
    data = frame.payload
    if len(data) < OFFLINE_LIST_HEADER_BYTES:
        raise ValueError(f"OFFLINE SESSION_LIST too short: {len(data)}")
    generation, cursor, total, count, next_cursor, stored_crc = struct.unpack(
        "<IIHHII", data[:OFFLINE_LIST_HEADER_BYTES]
    )
    if count > OFFLINE_LIST_MAX_ENTRIES:
        raise ValueError(f"OFFLINE SESSION_LIST count exceeds device maximum: {count}")
    expected_len = OFFLINE_LIST_HEADER_BYTES + count * OFFLINE_LIST_ENTRY_BYTES
    if len(data) != expected_len:
        raise ValueError(f"OFFLINE SESSION_LIST length mismatch: {len(data)} != {expected_len}")
    crc_input = bytearray(data)
    crc_input[16:20] = b"\x00\x00\x00\x00"
    actual_crc = crc32_ieee(crc_input)
    if actual_crc != stored_crc:
        raise ValueError(
            f"OFFLINE SESSION_LIST CRC mismatch: 0x{stored_crc:08X} != 0x{actual_crc:08X}"
        )
    if generation == 0:
        raise ValueError("OFFLINE SESSION_LIST generation is zero")
    if cursor > total or cursor + count > total:
        raise ValueError("OFFLINE SESSION_LIST cursor/count exceeds total")
    expected_next = 0xFFFFFFFF if cursor + count >= total else cursor + count
    if next_cursor != expected_next:
        raise ValueError(
            f"OFFLINE SESSION_LIST next cursor mismatch: {next_cursor} != {expected_next}"
        )
    if bool(frame.flags & OFFLINE_FLAG_LIST_LAST) != (next_cursor == 0xFFFFFFFF):
        raise ValueError("OFFLINE SESSION_LIST LIST_LAST flag mismatch")

    entries: list[OfflineSessionListEntry] = []
    for index in range(count):
        offset = OFFLINE_LIST_HEADER_BYTES + index * OFFLINE_LIST_ENTRY_BYTES
        values = struct.unpack("<IIBBHIIIQII", data[offset : offset + OFFLINE_LIST_ENTRY_BYTES])
        entry = OfflineSessionListEntry(*values)
        if entry.format_version not in OFFLINE_MANIFEST_SUPPORTED_VERSIONS:
            raise ValueError(f"OFFLINE list unsupported format version: {entry.format_version}")
        if entry.flags & ~0x07:
            raise ValueError(f"OFFLINE list entry flags are invalid: 0x{entry.flags:02X}")
        if entry.logical_bytes != entry.event_count * OFFLINE_EVENT_RECORD_BYTES:
            raise ValueError("OFFLINE list logical byte/event count mismatch")
        if entry.config_crc32 != OFFLINE_CONFIG_CRC32:
            raise ValueError(
                f"OFFLINE list config CRC mismatch: 0x{entry.config_crc32:08X}"
            )
        if not entry.finalized:
            raise ValueError(f"OFFLINE list contains non-final session state: {entry.state}")
        entries.append(entry)
    return OfflineSessionListPage(
        generation=generation,
        cursor=cursor,
        total=total,
        next_cursor=next_cursor,
        entries=tuple(entries),
        list_crc32=stored_crc,
    )


def parse_session_manifest(frame: OfflineOuterFrame) -> OfflineSessionManifest:
    if frame.frame_type != OFFLINE_FRAME_SESSION_BEGIN:
        raise ValueError("not an OFFLINE SESSION_BEGIN frame")
    if frame.flags != OFFLINE_FLAG_LAST:
        raise ValueError(f"OFFLINE SESSION_BEGIN invalid flags: 0x{frame.flags:02X}")
    data = frame.payload
    if len(data) < 14:
        raise ValueError(f"OFFLINE SESSION_BEGIN length error: {len(data)}")
    format_version = struct.unpack_from("<H", data, 12)[0]
    if format_version == OFFLINE_MANIFEST_LEGACY_VERSION:
        expected_bytes = OFFLINE_MANIFEST_LEGACY_BYTES
        crc_offset = OFFLINE_MANIFEST_LEGACY_CRC_OFFSET
        owner_user_id: int | None = None
    elif format_version == OFFLINE_MANIFEST_VERSION:
        expected_bytes = OFFLINE_MANIFEST_BYTES
        crc_offset = OFFLINE_MANIFEST_CRC_OFFSET
        owner_user_id = None
    else:
        raise ValueError(f"OFFLINE manifest unsupported version: {format_version}")
    if len(data) != expected_bytes:
        raise ValueError(f"OFFLINE manifest version/length mismatch: v{format_version} len={len(data)}")
    if format_version == OFFLINE_MANIFEST_VERSION:
        owner_user_id = struct.unpack_from(
            "<I", data, OFFLINE_MANIFEST_OWNER_USER_ID_OFFSET
        )[0]
        if owner_user_id == 0:
            raise ValueError("OFFLINE manifest v4 owner_user_id is zero")
    stored_crc = struct.unpack_from("<I", data, crc_offset)[0]
    crc_input = bytearray(data)
    crc_input[4:8] = b"\x00\x00\x00\x00"
    crc_input[crc_offset : crc_offset + 4] = b"\x00\x00\x00\x00"
    actual_crc = crc32_ieee(crc_input)
    if stored_crc != actual_crc:
        raise ValueError(
            f"OFFLINE manifest CRC mismatch: 0x{stored_crc:08X} != 0x{actual_crc:08X}"
        )
    session_id, transfer_id, generation = struct.unpack_from("<III", data, 0)
    format_version, record_bytes, slot_bytes, dimensions = struct.unpack_from("<HHHH", data, 12)
    q_frac_bits, flags = struct.unpack_from("<BB", data, 20)
    if data[22:24] != b"\x00\x00":
        raise ValueError("OFFLINE manifest reserved bytes are non-zero")
    sample_rate, event_count, logical_bytes, stream_crc = struct.unpack_from("<IIII", data, 24)
    start_unix_ms = struct.unpack_from("<Q", data, 40)[0]
    duration_ms = struct.unpack_from("<I", data, 48)[0]
    contract_crc = struct.unpack_from("<IIIII", data, 52)
    physical_span, extent_bytes = struct.unpack_from("<II", data, 72)
    if session_id == 0 or transfer_id == 0 or generation == 0:
        raise ValueError("OFFLINE manifest identity contains zero")
    if record_bytes != OFFLINE_EVENT_RECORD_BYTES or slot_bytes != OFFLINE_EVENT_SLOT_BYTES:
        raise ValueError("OFFLINE manifest event size contract mismatch")
    if dimensions != OFFLINE_EVENT_DIMENSIONS or q_frac_bits != OFFLINE_Q_FRAC_BITS:
        raise ValueError("OFFLINE manifest feature contract mismatch")
    if flags & ~0x03:
        raise ValueError(f"OFFLINE manifest unsupported flags: 0x{flags:02X}")
    if sample_rate != OFFLINE_SAMPLE_RATE_HZ:
        raise ValueError(f"OFFLINE manifest sample rate mismatch: {sample_rate}")
    if logical_bytes != event_count * record_bytes:
        raise ValueError("OFFLINE manifest logical byte/event count mismatch")
    if extent_bytes < physical_span:
        raise ValueError("OFFLINE manifest extent smaller than physical span")
    if contract_crc != OFFLINE_CONTRACT_CRC32:
        raise ValueError(
            "OFFLINE manifest algorithm contract CRC mismatch: "
            + ",".join(f"0x{value:08X}" for value in contract_crc)
        )
    stop_reason = struct.unpack_from("<H", data, 80)[0]
    clean = bool(data[82])
    health = data[83]
    quality = struct.unpack_from("<IIIIII", data, 84)
    if health not in OFFLINE_SESSION_HEALTH_NAMES:
        raise ValueError(f"OFFLINE manifest health unsupported: {health}")
    if data[105:108] != b"\x00\x00\x00":
        raise ValueError("OFFLINE manifest reserved bytes are non-zero")
    shock_id_crc32, config_crc32 = struct.unpack_from("<II", data, 108)
    if shock_id_crc32 != OFFLINE_SHOCK_ID_CRC32:
        raise ValueError(
            f"OFFLINE manifest shock contract CRC mismatch: 0x{shock_id_crc32:08X}"
        )
    if config_crc32 != OFFLINE_CONFIG_CRC32:
        raise ValueError(
            f"OFFLINE manifest config CRC mismatch: 0x{config_crc32:08X}"
        )
    return OfflineSessionManifest(
        session_id=session_id,
        transfer_id=transfer_id,
        generation=generation,
        format_version=format_version,
        event_record_bytes=record_bytes,
        event_slot_bytes=slot_bytes,
        feature_dimensions=dimensions,
        q_frac_bits=q_frac_bits,
        flags=flags,
        sample_rate_hz=sample_rate,
        event_count=event_count,
        logical_bytes=logical_bytes,
        stream_crc32=stream_crc,
        start_unix_ms=start_unix_ms,
        duration_ms=duration_ms,
        contract_crc32=contract_crc,
        physical_span_bytes=physical_span,
        extent_bytes=extent_bytes,
        shock_id_crc32=shock_id_crc32,
        config_crc32=config_crc32,
        stop_reason=stop_reason,
        clean=clean,
        health=health,
        fifo_overflow_count=quality[0],
        fifo_discard_count=quality[1],
        time_gap_count=quality[2],
        feature_drop_count=quality[3],
        q12_clip_count=quality[4],
        flash_error_count=quality[5],
        manifest_crc32=stored_crc,
        owner_user_id=owner_user_id,
    )


def parse_session_chunk(frame: OfflineOuterFrame) -> OfflineSessionChunk:
    if frame.frame_type != OFFLINE_FRAME_SESSION_CHUNK:
        raise ValueError("not an OFFLINE SESSION_CHUNK frame")
    if frame.flags & ~(OFFLINE_FLAG_LAST | OFFLINE_FLAG_RETRANSMIT):
        raise ValueError(f"OFFLINE SESSION_CHUNK invalid flags: 0x{frame.flags:02X}")
    data = frame.payload
    if len(data) < OFFLINE_CHUNK_HEADER_BYTES:
        raise ValueError(f"OFFLINE SESSION_CHUNK too short: {len(data)}")
    session_id, transfer_id, offset, total_bytes, data_len, reserved, data_crc, prefix_crc = struct.unpack(
        "<IIIIHHII", data[:OFFLINE_CHUNK_HEADER_BYTES]
    )
    if reserved != 0:
        raise ValueError("OFFLINE SESSION_CHUNK reserved field is non-zero")
    if data_len == 0 or data_len > OFFLINE_CHUNK_DATA_MAX_BYTES:
        raise ValueError(f"OFFLINE SESSION_CHUNK data length error: {data_len}")
    if len(data) != OFFLINE_CHUNK_HEADER_BYTES + data_len:
        raise ValueError("OFFLINE SESSION_CHUNK payload length mismatch")
    payload = data[OFFLINE_CHUNK_HEADER_BYTES:]
    actual_crc = crc32_ieee(payload)
    if data_crc != actual_crc:
        raise ValueError(
            f"OFFLINE SESSION_CHUNK CRC mismatch: 0x{data_crc:08X} != 0x{actual_crc:08X}"
        )
    if offset + data_len > total_bytes:
        raise ValueError("OFFLINE SESSION_CHUNK exceeds total bytes")
    if bool(frame.flags & OFFLINE_FLAG_LAST) != (offset + data_len == total_bytes):
        raise ValueError("OFFLINE SESSION_CHUNK LAST flag does not match total bytes")
    return OfflineSessionChunk(
        session_id=session_id,
        transfer_id=transfer_id,
        offset=offset,
        total_bytes=total_bytes,
        data_crc32=data_crc,
        prefix_crc32=prefix_crc,
        data=payload,
    )


def parse_session_end(frame: OfflineOuterFrame) -> OfflineSessionEnd:
    if frame.frame_type != OFFLINE_FRAME_SESSION_END:
        raise ValueError("not an OFFLINE SESSION_END frame")
    if frame.flags != OFFLINE_FLAG_LAST:
        raise ValueError(f"OFFLINE SESSION_END invalid flags: 0x{frame.flags:02X}")
    if len(frame.payload) != OFFLINE_END_BYTES:
        raise ValueError(f"OFFLINE SESSION_END length error: {len(frame.payload)}")
    values = struct.unpack("<IIIIIIIIIII", frame.payload)
    if values[-1] != 0:
        raise ValueError("OFFLINE SESSION_END reserved field is non-zero")
    end = OfflineSessionEnd(*values[:-1])
    if end.logical_bytes != end.event_count * OFFLINE_EVENT_RECORD_BYTES:
        raise ValueError("OFFLINE SESSION_END logical byte/event count mismatch")
    if end.extent_bytes < end.physical_span_bytes:
        raise ValueError("OFFLINE SESSION_END extent smaller than physical span")
    return end


def parse_reclaim_status(frame: OfflineOuterFrame) -> OfflineReclaimStatus:
    if frame.frame_type != OFFLINE_FRAME_RECLAIM_STATUS:
        raise ValueError("not an OFFLINE RECLAIM_STATUS frame")
    if frame.flags != OFFLINE_FLAG_LAST:
        raise ValueError(f"OFFLINE RECLAIM_STATUS invalid flags: 0x{frame.flags:02X}")
    if len(frame.payload) != OFFLINE_RECLAIM_BYTES:
        raise ValueError(f"OFFLINE RECLAIM_STATUS length error: {len(frame.payload)}")
    session_id, generation = struct.unpack_from("<II", frame.payload, 0)
    state = frame.payload[8]
    if frame.payload[9:12] != b"\x00\x00\x00":
        raise ValueError("OFFLINE RECLAIM_STATUS reserved bytes are non-zero")
    erased_bytes, extent_bytes, reserved = struct.unpack_from("<III", frame.payload, 12)
    if reserved != 0:
        raise ValueError("OFFLINE RECLAIM_STATUS reserved field is non-zero")
    if state not in RECLAIM_STATE_NAMES:
        raise ValueError(f"OFFLINE RECLAIM_STATUS unknown state: {state}")
    if erased_bytes > extent_bytes:
        raise ValueError("OFFLINE RECLAIM_STATUS erased bytes exceed extent")
    return OfflineReclaimStatus(session_id, generation, state, erased_bytes, extent_bytes)


def parse_sync_abort(frame: OfflineOuterFrame) -> OfflineSyncAbort:
    if frame.frame_type != OFFLINE_FRAME_SYNC_ABORT:
        raise ValueError("not an OFFLINE SYNC_ABORT frame")
    if frame.flags != OFFLINE_FLAG_LAST:
        raise ValueError(f"OFFLINE SYNC_ABORT invalid flags: 0x{frame.flags:02X}")
    if len(frame.payload) != OFFLINE_ABORT_BYTES:
        raise ValueError(f"OFFLINE SYNC_ABORT length error: {len(frame.payload)}")
    return OfflineSyncAbort(*struct.unpack("<IIIIII", frame.payload))


def parse_event_record(data: bytes | bytearray, *, expected_session_id: int | None = None,
                       expected_event_index: int | None = None,
                       expected_config_crc32: int = OFFLINE_CONFIG_CRC32) -> OfflineEventRecord:
    raw = bytes(data)
    if len(raw) != OFFLINE_EVENT_RECORD_BYTES:
        raise ValueError(f"OFFLINE event length error: {len(raw)}")
    magic = struct.unpack_from("<I", raw, 0)[0]
    version = raw[4]
    header_bytes = raw[5]
    if magic != OFFLINE_EVENT_MAGIC:
        raise ValueError(f"OFFLINE event magic error: 0x{magic:08X}")
    if version != OFFLINE_EVENT_VERSION or header_bytes != OFFLINE_EVENT_HEADER_BYTES:
        raise ValueError("OFFLINE event version/header contract mismatch")
    flags = struct.unpack_from("<H", raw, 6)[0]
    if flags & ~OFFLINE_EVENT_FLAGS_MASK:
        raise ValueError(f"OFFLINE event reserved flags are non-zero: 0x{flags:04X}")
    session_id, event_index, center, peak, config_crc = struct.unpack_from("<IIIII", raw, 8)
    shock_numerator, shock_baseline, shock_ratio = struct.unpack_from("<fff", raw, 28)
    versions = struct.unpack_from("<BBBB", raw, 40)
    stored_crc = struct.unpack_from("<I", raw, OFFLINE_EVENT_CRC_OFFSET)[0]
    if expected_session_id is not None and session_id != (expected_session_id & 0xFFFFFFFF):
        raise ValueError(f"OFFLINE event session mismatch: {session_id} != {expected_session_id}")
    if expected_event_index is not None and event_index != expected_event_index:
        raise ValueError(f"OFFLINE event index mismatch: {event_index} != {expected_event_index}")
    if config_crc != (expected_config_crc32 & 0xFFFFFFFF):
        raise ValueError(
            "OFFLINE event config CRC mismatch: "
            f"0x{config_crc:08X} != 0x{expected_config_crc32 & 0xFFFFFFFF:08X}"
        )
    if not all(math.isfinite(value) for value in (shock_numerator, shock_baseline, shock_ratio)):
        raise ValueError("OFFLINE event shock evidence is non-finite")
    if versions != OFFLINE_EVENT_VERSIONS:
        raise ValueError(
            "OFFLINE event stage version mismatch: "
            + ",".join(str(value) for value in versions)
        )
    crc_input = bytearray(raw)
    crc_input[OFFLINE_EVENT_CRC_OFFSET : OFFLINE_EVENT_CRC_OFFSET + 4] = b"\x00\x00\x00\x00"
    actual_crc = crc32_ieee(crc_input)
    if stored_crc != actual_crc:
        raise ValueError(
            f"OFFLINE event record CRC mismatch: 0x{stored_crc:08X} != 0x{actual_crc:08X}"
        )
    features = struct.unpack_from("<128h", raw, OFFLINE_EVENT_HEADER_BYTES)
    return OfflineEventRecord(
        flags=flags,
        session_id=session_id,
        event_index=event_index,
        center_sample_index=center,
        peak_mag2_raw=peak,
        config_crc32=config_crc,
        shock_numerator=shock_numerator,
        shock_baseline=shock_baseline,
        shock_ratio_rounded_4dp=shock_ratio,
        detector_version=versions[0],
        shock_version=versions[1],
        feature_version=versions[2],
        codec_version=versions[3],
        record_crc32=stored_crc,
        feature_q12=features,
    )


def build_session_list_command(seq: int, *, generation: int = 0, cursor: int = 0,
                               page_size: int = 0) -> bytes:
    return build_zy100_command(
        CMD_OFFLINE_SESSION_LIST,
        seq=seq,
        user_id=generation,
        device_time_ms=cursor & 0xFFFFFFFF,
        training_id=page_size,
    )


def build_session_begin_command(seq: int, *, session_id: int, generation: int) -> bytes:
    return build_zy100_command(
        CMD_OFFLINE_SESSION_BEGIN,
        seq=seq,
        user_id=session_id,
        device_time_ms=(generation & 0xFFFFFFFF) << 32,
        training_id=0,
    )


def build_chunk_ack_command(seq: int, *, session_id: int, transfer_id: int,
                            next_offset: int, prefix_crc32: int) -> bytes:
    return build_zy100_command(
        CMD_OFFLINE_CHUNK_ACK,
        seq=seq,
        user_id=session_id,
        device_time_ms=((transfer_id & 0xFFFFFFFF) << 32) | (next_offset & 0xFFFFFFFF),
        training_id=prefix_crc32,
    )


def build_session_resume_command(seq: int, *, session_id: int, transfer_id: int,
                                 next_offset: int, prefix_crc32: int) -> bytes:
    return build_zy100_command(
        CMD_OFFLINE_SESSION_RESUME,
        seq=seq,
        user_id=session_id,
        device_time_ms=((transfer_id & 0xFFFFFFFF) << 32) | (next_offset & 0xFFFFFFFF),
        training_id=prefix_crc32,
    )


def build_final_confirm_command(seq: int, *, session_id: int, generation: int,
                                transfer_id: int, stream_crc32: int) -> bytes:
    return build_zy100_command(
        CMD_OFFLINE_FINAL_CONFIRM,
        seq=seq,
        user_id=session_id,
        device_time_ms=((generation & 0xFFFFFFFF) << 32) | (transfer_id & 0xFFFFFFFF),
        training_id=stream_crc32,
    )


def build_reclaim_status_command(seq: int, *, session_id: int, generation: int) -> bytes:
    return build_zy100_command(
        CMD_OFFLINE_RECLAIM_STATUS,
        seq=seq,
        user_id=session_id,
        device_time_ms=(generation & 0xFFFFFFFF) << 32,
        training_id=0,
    )
