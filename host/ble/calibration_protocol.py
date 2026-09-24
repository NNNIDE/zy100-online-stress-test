from __future__ import annotations

import binascii
import math
import struct
from dataclasses import dataclass
from typing import Final


CALIBRATION_SERVICE_UUID: Final[str] = "9eca3000-4a9d-4f4d-b61a-7d0f5a9a1000"
CALIBRATION_INFO_UUID: Final[str] = "9eca3001-4a9d-4f4d-b61a-7d0f5a9a1000"
CALIBRATION_TX_UUID: Final[str] = "9eca3002-4a9d-4f4d-b61a-7d0f5a9a1000"
CALIBRATION_RX_UUID: Final[str] = "9eca3003-4a9d-4f4d-b61a-7d0f5a9a1000"
CALIBRATION_STATUS_UUID: Final[str] = "9eca3004-4a9d-4f4d-b61a-7d0f5a9a1000"

CAL_PROTOCOL_VERSION: Final[int] = 1
CAL_RX_MAGIC: Final[int] = 0xC3
CAL_TX_MAGIC: Final[int] = 0xD3
CAL_RX_HEADER_BYTES: Final[int] = 14
CAL_TX_HEADER_BYTES: Final[int] = 16
CAL_INFO_BYTES: Final[int] = 20
CAL_STATUS_BYTES: Final[int] = 16
CAL_CACHE_CONFIRM_BYTES: Final[int] = 12

CAL_RX_ACK: Final[int] = 0x04
CAL_RX_REQUEST: Final[int] = 0x05
CAL_RX_START_MAG_CAL: Final[int] = 0x10
CAL_RX_REQUEST_DIAGNOSTICS: Final[int] = 0x11
CAL_RX_START_MAG_CAL_WITH_POINTS: Final[int] = 0x12
CAL_RX_CONFIRM_INFO: Final[int] = 0x13
CAL_RX_CONFIRM_CACHED_RECORD: Final[int] = 0x14

CAL_TX_START: Final[int] = 0x01
CAL_TX_DATA: Final[int] = 0x02
CAL_TX_END: Final[int] = 0x03
CAL_TX_DIAG_START: Final[int] = 0x04
CAL_TX_DIAG_DATA: Final[int] = 0x05
CAL_TX_DIAG_END: Final[int] = 0x06

CAL_RECORD_MAGIC: Final[int] = 0x314C4143
CAL_RECORD_VERSION: Final[int] = 1
CAL_RECORD_HEADER_BYTES: Final[int] = 30
CAL_RECORD_CRC_OFFSET: Final[int] = 26
CAL_RECORD_MAX_BYTES: Final[int] = 256
CAL_MAG_DIAG_MAGIC: Final[int] = 0x3144474D
CAL_MAG_DIAG_VERSION: Final[int] = 1
CAL_MAG_DIAG_BYTES: Final[int] = 68
CAL_MAG_DIAG_CRC_OFFSET: Final[int] = 64

MAG_DIAG_COMPLETION_NAMES: Final[dict[int, str]] = {
    0: "NONE",
    1: "SUCCESS",
    2: "LIMITED",
    3: "FAILED",
    4: "NO_UPDATE",
}
MAG_DIAG_MODEL_NAMES: Final[dict[int, str]] = {
    0: "NONE",
    1: "HARD_IRON",
    2: "AXIS_ALIGNED",
    3: "FULL",
}
MAG_DIAG_READY_NAMES: Final[dict[int, str]] = {
    0: "READY",
    1: "INVALID_ARGUMENT",
    2: "INSUFFICIENT_SAMPLES",
    3: "INSUFFICIENT_COVERAGE",
    4: "INSUFFICIENT_AXIS_SPAN",
}
MAG_DIAG_SOLVE_NAMES: Final[dict[int, str]] = {
    0: "OK",
    1: "INVALID_ARGUMENT",
    2: "NOT_READY",
    3: "LINEAR_SYSTEM",
    4: "SHAPE_INVERSE",
    5: "NON_POSITIVE_SCALE",
    6: "NON_POSITIVE_ELLIPSOID",
    7: "INVALID_NORMALIZATION",
    8: "NON_FINITE_RESULT",
    0xFF: "NOT_ATTEMPTED",
}

CAL_VALID_IMU_GYRO: Final[int] = 0x00000001
CAL_VALID_IMU_ACCEL: Final[int] = 0x00000002
CAL_VALID_IMU_INSTALL: Final[int] = 0x00000004
CAL_VALID_MAG_MODEL: Final[int] = 0x00000008
CAL_VALID_MAG_INSTALL: Final[int] = 0x00000010
CAL_MAG_MODEL_AXIS_ALIGNED: Final[int] = 0x00010000
CAL_MAG_MODEL_HARD_IRON_ONLY: Final[int] = 0x00020000
CAL_MAG_MODEL_FLAG_MASK: Final[int] = 0x00030000

CAL_TLV_IMU_GYRO_MODEL: Final[int] = 0x01
CAL_TLV_IMU_ACCEL_MODEL: Final[int] = 0x02
CAL_TLV_IMU_INSTALL_MATRIX: Final[int] = 0x03
CAL_TLV_MAG_MODEL: Final[int] = 0x04
CAL_TLV_MAG_INSTALL_MATRIX: Final[int] = 0x05
CAL_TLV_QUALITY_META: Final[int] = 0x06

CAL_STATUS_TEXT: Final[dict[int, str]] = {
    0: "NO_DATA",
    1: "READY",
    2: "SENDING",
    3: "WAIT_ACK",
    4: "ACKED",
    5: "WRITE_RECEIVING",
    6: "COMMIT_PENDING",
    7: "WRITE_OK",
    8: "BAD_FRAME",
    9: "BAD_RECORD",
    10: "BUSY",
    11: "NOT_PAIRED",
    12: "STORAGE_ERROR",
    13: "TX_ERROR",
    14: "CAL_ACCEPTED",
    15: "CAL_STARTING",
    16: "CAL_COLLECTING",
    17: "CAL_FITTING",
    18: "CAL_SAVING",
    19: "CAL_SUCCESS",
    20: "CAL_FAILED_TIMEOUT",
    21: "CAL_FAILED_COVERAGE",
    22: "CAL_FAILED_FIT",
    23: "CAL_FAILED_SENSOR",
    24: "CAL_SHUTDOWN_PENDING",
    25: "INVALID_STATE",
    26: "CAL_SUCCESS_LIMITED",
    27: "CAL_COMPLETED_NO_UPDATE",
    28: "CAL_INFO_CONFIRMED",
    29: "CAL_INFO_MISMATCH",
    30: "CAL_INFO_REQUIRED",
    31: "CAL_RECORD_CACHE_CONFIRMED",
}

CAL_STATUS_ZH: Final[dict[int, str]] = {
    0: "无校准记录",
    1: "设备就绪",
    2: "正在发送记录",
    3: "等待上位机确认",
    4: "上位机已确认",
    5: "正在接收参数",
    6: "等待提交参数",
    7: "参数写入成功",
    8: "协议帧错误",
    9: "校准记录无效",
    10: "设备忙",
    11: "连接未加密",
    12: "Flash保存失败",
    13: "记录发送失败",
    14: "校准已接受",
    15: "传感器启动中",
    16: "正在采集地磁",
    17: "正在拟合校准参数",
    18: "正在保存到设备",
    19: "设备校准并保存成功",
    20: "校准超时",
    21: "空间方向覆盖不足",
    22: "椭球拟合失败",
    23: "地磁传感器失败",
    24: "保存完成后关机",
    25: "设备状态不允许校准",
    26: "校准完成，有限模型可用",
    27: "校准完成，本次未更新参数",
    31: "本地校准缓存已由设备确认",
}

CAL_FAILURE_DETAIL_ZH: Final[dict[int, str]] = {
    1: "有效样本数量不足",
    2: "三维方向覆盖不足",
    3: "至少一个轴没有形成有效跨度",
    4: "正规方程奇异或条件数过大",
    5: "椭球形状矩阵不可逆",
    6: "椭球尺度不是正数",
    7: "椭球矩阵不是正定矩阵",
    8: "soft-iron矩阵归一化失败",
    9: "结果包含NaN或Inf",
    10: "校准记录编码或校验失败",
    11: "没有获得有效地磁样本",
    12: "本次候选参数不优于设备已有记录",
}

CAL_ACTIVE_STATUSES: Final[frozenset[int]] = frozenset({14, 15, 16, 17, 18, 24})
CAL_FAILURE_STATUSES: Final[frozenset[int]] = frozenset({8, 9, 10, 11, 12, 13, 20, 21, 22, 23, 25})
CAL_SUCCESS_STATUSES: Final[frozenset[int]] = frozenset({19, 26, 27})


@dataclass(frozen=True, slots=True)
class CalibrationInfo:
    valid: bool
    record_version: int
    generation: int
    valid_flags: int
    record_bytes: int
    quality: int
    crc32: int

    @property
    def mag_model_kind(self) -> str:
        return mag_model_kind_from_flags(self.valid_flags)

    def to_dict(self) -> dict[str, int | bool]:
        return {
            "valid": self.valid,
            "record_version": self.record_version,
            "generation": self.generation,
            "valid_flags": self.valid_flags,
            "record_bytes": self.record_bytes,
            "quality": self.quality,
            "crc32": self.crc32,
            "mag_model_kind": self.mag_model_kind,
        }


@dataclass(frozen=True, slots=True)
class CalibrationStatus:
    code: int
    transaction_id: int
    detail: int
    generation: int
    crc32: int

    @property
    def name(self) -> str:
        return calibration_status_name(self.code)

    def to_dict(self) -> dict[str, int | str]:
        return {
            "code": self.code,
            "name": self.name,
            "transaction_id": self.transaction_id,
            "detail": self.detail,
            "generation": self.generation,
            "crc32": self.crc32,
        }


@dataclass(frozen=True, slots=True)
class CalibrationTxFrame:
    frame_type: int
    transaction_id: int
    flags: int
    offset: int
    payload: bytes
    total_len: int
    crc32: int


@dataclass(frozen=True, slots=True)
class CalibrationRecord:
    generation: int
    valid_flags: int
    quality: int
    created_unix_ms: int
    record_bytes: int
    crc32: int
    tlvs: tuple[tuple[int, bytes], ...]
    mag_bias: tuple[float, float, float] | None
    mag_matrix: tuple[float, ...] | None
    raw: bytes

    @property
    def mag_model_kind(self) -> str:
        return mag_model_kind_from_flags(self.valid_flags)

    def to_dict(self) -> dict[str, object]:
        return {
            "format_version": CAL_RECORD_VERSION,
            "generation": self.generation,
            "valid_flags": self.valid_flags,
            "quality": self.quality,
            "mag_model_kind": self.mag_model_kind,
            "created_unix_ms": self.created_unix_ms,
            "record_bytes": self.record_bytes,
            "crc32": f"0x{self.crc32:08X}",
            "tlvs": [{"type": t, "length": len(value)} for t, value in self.tlvs],
            "mag_bias_raw_counts": list(self.mag_bias) if self.mag_bias is not None else None,
            "mag_soft_iron_matrix_row_major": list(self.mag_matrix) if self.mag_matrix is not None else None,
        }


@dataclass(frozen=True, slots=True)
class MagCalibrationDiagnostics:
    transaction_id: int
    completion_status: int
    final_model: int
    sample_count: int
    rejected_sample_count: int
    sensor_read_errors: int
    elapsed_ms: int
    fit_attempts: int
    coverage_mask: int
    direction_mask: int
    final_ready_status: int
    full_solve_status: int
    axis_solve_status: int
    hard_solve_status: int
    quality: int
    rms_x1e6: int
    min_values: tuple[int, int, int]
    max_values: tuple[int, int, int]
    crc32: int
    raw: bytes

    @property
    def completion_name(self) -> str:
        return MAG_DIAG_COMPLETION_NAMES.get(
            self.completion_status, f"UNKNOWN_0x{self.completion_status:02X}"
        )

    @property
    def final_model_name(self) -> str:
        return MAG_DIAG_MODEL_NAMES.get(
            self.final_model, f"UNKNOWN_0x{self.final_model:02X}"
        )

    @property
    def final_ready_name(self) -> str:
        return MAG_DIAG_READY_NAMES.get(
            self.final_ready_status, f"UNKNOWN_0x{self.final_ready_status:02X}"
        )

    @staticmethod
    def solve_name(value: int) -> str:
        return MAG_DIAG_SOLVE_NAMES.get(value, f"UNKNOWN_0x{value:02X}")

    def to_dict(self) -> dict[str, object]:
        spans = [
            int(self.max_values[index]) - int(self.min_values[index])
            for index in range(3)
        ]
        return {
            "format": "MGD1",
            "version": CAL_MAG_DIAG_VERSION,
            "calibration_transaction_id": self.transaction_id,
            "completion_status": self.completion_status,
            "completion_name": self.completion_name,
            "final_model": self.final_model,
            "final_model_name": self.final_model_name,
            "sample_count": self.sample_count,
            "rejected_sample_count": self.rejected_sample_count,
            "sensor_read_errors": self.sensor_read_errors,
            "elapsed_ms": self.elapsed_ms,
            "fit_attempts": self.fit_attempts,
            "coverage_mask": self.coverage_mask,
            "direction_mask": self.direction_mask,
            "final_ready_status": self.final_ready_status,
            "final_ready_name": self.final_ready_name,
            "full_solve_status": self.full_solve_status,
            "full_solve_name": self.solve_name(self.full_solve_status),
            "axis_solve_status": self.axis_solve_status,
            "axis_solve_name": self.solve_name(self.axis_solve_status),
            "hard_solve_status": self.hard_solve_status,
            "hard_solve_name": self.solve_name(self.hard_solve_status),
            "quality": self.quality,
            "rms_x1e6": self.rms_x1e6,
            "rms": self.rms_x1e6 / 1_000_000.0,
            "min_values": list(self.min_values),
            "max_values": list(self.max_values),
            "spans": spans,
            "crc32": f"0x{self.crc32:08X}",
        }


@dataclass(frozen=True, slots=True)
class CalibrationReceiverEvent:
    kind: str
    transaction_id: int = 0
    received_bytes: int = 0
    total_bytes: int = 0
    record: CalibrationRecord | None = None
    diagnostics: MagCalibrationDiagnostics | None = None
    transfer_kind: str = ""


def calibration_status_name(code: int) -> str:
    return CAL_STATUS_TEXT.get(code & 0xFF, f"UNKNOWN_0x{code & 0xFF:02X}")


def mag_model_kind_from_flags(valid_flags: int) -> str:
    flags = valid_flags & CAL_MAG_MODEL_FLAG_MASK
    if flags == CAL_MAG_MODEL_AXIS_ALIGNED:
        return "LIMITED_AXIS"
    if flags == CAL_MAG_MODEL_HARD_IRON_ONLY:
        return "HARD_IRON"
    if valid_flags & CAL_VALID_MAG_MODEL:
        return "FULL"
    return "NONE"


def calibration_status_zh(code: int) -> str:
    return CAL_STATUS_ZH.get(code & 0xFF, f"未知状态 0x{code & 0xFF:02X}")


def calibration_failure_detail_zh(detail: int) -> str:
    return CAL_FAILURE_DETAIL_ZH.get(detail & 0xFFFF, f"原因码 {detail & 0xFFFF}")


def calibration_completion_detail_zh(detail: int) -> str:
    category = detail & 0xFF00
    reason = detail & 0x00FF
    if detail == 0x0101:
        return "仅硬铁偏置模型可用"
    if detail == 0x0102:
        return "轴对齐软铁模型可用"
    reason_text = CAL_FAILURE_DETAIL_ZH.get(reason, f"原因码 {reason}")
    if category == 0x0200:
        return f"本次未覆盖，继续沿用设备已有参数；{reason_text}"
    if category == 0x0300:
        return f"本次未生成安全可用参数；{reason_text}"
    return f"完成详情 0x{detail & 0xFFFF:04X}"


def build_calibration_rx_frame(
    opcode: int,
    transaction_id: int,
    *,
    offset: int = 0,
    payload: bytes = b"",
    total_len: int = 0,
    crc32: int = 0,
) -> bytes:
    if not (0 <= offset <= 0xFFFF and 0 <= total_len <= 0xFFFF):
        raise ValueError("Calibration RX offset/total_len out of range")
    payload = bytes(payload)
    if len(payload) > 0xFFFF:
        raise ValueError("Calibration RX payload too large")
    return struct.pack(
        "<BBBBHHHI",
        CAL_RX_MAGIC,
        CAL_PROTOCOL_VERSION,
        opcode & 0xFF,
        transaction_id & 0xFF,
        offset,
        len(payload),
        total_len,
        crc32 & 0xFFFFFFFF,
    ) + payload


def build_start_mag_calibration(transaction_id: int) -> bytes:
    return build_calibration_rx_frame(CAL_RX_START_MAG_CAL, transaction_id)


def build_calibration_ack(transaction_id: int, crc32: int) -> bytes:
    return build_calibration_rx_frame(CAL_RX_ACK, transaction_id, crc32=crc32)


def build_calibration_request(transaction_id: int) -> bytes:
    return build_calibration_rx_frame(CAL_RX_REQUEST, transaction_id)


def build_calibration_diagnostics_request(transaction_id: int) -> bytes:
    return build_calibration_rx_frame(CAL_RX_REQUEST_DIAGNOSTICS, transaction_id)


def build_calibration_info_confirmation(
    transaction_id: int, raw_info: bytes | bytearray
) -> bytes:
    payload = bytes(raw_info)
    if len(payload) != CAL_INFO_BYTES:
        raise ValueError(f"Calibration Info confirmation must be {CAL_INFO_BYTES} bytes")
    return build_calibration_rx_frame(
        CAL_RX_CONFIRM_INFO,
        transaction_id,
        payload=payload,
        total_len=CAL_INFO_BYTES,
        crc32=binascii.crc32(payload) & 0xFFFFFFFF,
    )


def build_calibration_cached_record_confirmation(
    transaction_id: int,
    *,
    generation: int,
    valid_flags: int,
    record_bytes: int,
    record_version: int,
    crc32: int,
) -> bytes:
    if not (0 <= record_bytes <= 0xFFFF):
        raise ValueError("Calibration cached record length out of range")
    payload = struct.pack(
        "<IIHBB",
        generation & 0xFFFFFFFF,
        valid_flags & 0xFFFFFFFF,
        record_bytes,
        record_version & 0xFF,
        0,
    )
    assert len(payload) == CAL_CACHE_CONFIRM_BYTES
    return build_calibration_rx_frame(
        CAL_RX_CONFIRM_CACHED_RECORD,
        transaction_id,
        payload=payload,
        total_len=CAL_CACHE_CONFIRM_BYTES,
        crc32=crc32,
    )


def build_full_mag_calibration_record(
    current: CalibrationRecord,
    *,
    generation: int,
    quality: int,
    created_unix_ms: int,
    bias_raw_counts: tuple[float, float, float],
    matrix_row_major: tuple[float, ...],
) -> CalibrationRecord:
    if len(bias_raw_counts) != 3 or len(matrix_row_major) != 9:
        raise ValueError("FULL mag model requires 3 bias and 9 matrix values")
    values = tuple(bias_raw_counts) + tuple(matrix_row_major)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("FULL mag model contains NaN/Inf")
    tlvs: list[tuple[int, bytes]] = []
    replaced = False
    mag_value = struct.pack("<12f", *values)
    for tlv_type, value in current.tlvs:
        if tlv_type == CAL_TLV_MAG_MODEL:
            tlvs.append((tlv_type, mag_value))
            replaced = True
        else:
            tlvs.append((tlv_type, value))
    if not replaced:
        tlvs.append((CAL_TLV_MAG_MODEL, mag_value))
    body = b"".join(bytes((tlv_type, len(value))) + value for tlv_type, value in tlvs)
    record_bytes = CAL_RECORD_HEADER_BYTES + len(body)
    if record_bytes > CAL_RECORD_MAX_BYTES:
        raise ValueError("Calibration record is too large")
    valid_flags = (
        current.valid_flags
        | CAL_VALID_MAG_MODEL
    ) & ~CAL_MAG_MODEL_FLAG_MASK
    raw = bytearray(
        struct.pack(
            "<IBBHIIHQI",
            CAL_RECORD_MAGIC,
            CAL_RECORD_VERSION,
            CAL_RECORD_HEADER_BYTES,
            record_bytes,
            generation & 0xFFFFFFFF,
            valid_flags,
            max(0, min(0xFFFF, int(quality))),
            created_unix_ms & 0xFFFFFFFFFFFFFFFF,
            0,
        )
        + body
    )
    struct.pack_into("<I", raw, CAL_RECORD_CRC_OFFSET, _record_crc32(bytes(raw)))
    return parse_calibration_record(raw)


def parse_calibration_info(data: bytes | bytearray) -> CalibrationInfo:
    raw = bytes(data)
    if len(raw) != CAL_INFO_BYTES:
        raise ValueError(f"Calibration Info must be {CAL_INFO_BYTES} bytes, got {len(raw)}")
    magic, version, record_version, valid, generation, valid_flags, record_bytes, quality, crc32 = struct.unpack(
        "<BBBBIIHHI", raw
    )
    if magic != CAL_TX_MAGIC or version != CAL_PROTOCOL_VERSION:
        raise ValueError("Calibration Info magic/version mismatch")
    if valid not in (0, 1):
        raise ValueError("Calibration Info valid flag is invalid")
    if valid and (record_version != CAL_RECORD_VERSION or not (CAL_RECORD_HEADER_BYTES <= record_bytes <= 256)):
        raise ValueError("Calibration Info record metadata is invalid")
    if not valid and any((record_version, generation, valid_flags, record_bytes, quality, crc32)):
        raise ValueError("Uncalibrated Calibration Info fields must be zero")
    return CalibrationInfo(bool(valid), record_version, generation, valid_flags, record_bytes, quality, crc32)


def parse_calibration_status(data: bytes | bytearray) -> CalibrationStatus:
    raw = bytes(data)
    if len(raw) != CAL_STATUS_BYTES:
        raise ValueError(f"Calibration Status must be {CAL_STATUS_BYTES} bytes, got {len(raw)}")
    magic, version, code, transaction_id, detail, reserved, generation, crc32 = struct.unpack("<BBBBHHII", raw)
    if magic != CAL_TX_MAGIC or version != CAL_PROTOCOL_VERSION or reserved != 0:
        raise ValueError("Calibration Status header is invalid")
    return CalibrationStatus(code, transaction_id, detail, generation, crc32)


def parse_calibration_tx_frame(data: bytes | bytearray) -> CalibrationTxFrame:
    raw = bytes(data)
    if len(raw) < CAL_TX_HEADER_BYTES:
        raise ValueError("Calibration TX frame is shorter than header")
    magic, version, frame_type, transaction_id, flags, reserved, offset, payload_len, total_len, crc32 = struct.unpack(
        "<BBBBBBHHHI", raw[:CAL_TX_HEADER_BYTES]
    )
    if magic != CAL_TX_MAGIC or version != CAL_PROTOCOL_VERSION or reserved != 0:
        raise ValueError("Calibration TX header is invalid")
    if flags & ~0x01:
        raise ValueError("Calibration TX flags are invalid")
    if frame_type not in (
        CAL_TX_START,
        CAL_TX_DATA,
        CAL_TX_END,
        CAL_TX_DIAG_START,
        CAL_TX_DIAG_DATA,
        CAL_TX_DIAG_END,
    ):
        raise ValueError(f"Unknown Calibration TX frame type: {frame_type}")
    if len(raw) != CAL_TX_HEADER_BYTES + payload_len:
        raise ValueError("Calibration TX payload length mismatch")
    if total_len > CAL_RECORD_MAX_BYTES or offset > total_len or offset + payload_len > total_len:
        raise ValueError("Calibration TX range is invalid")
    if frame_type not in (CAL_TX_DATA, CAL_TX_DIAG_DATA) and payload_len != 0:
        raise ValueError("Calibration START/END must not carry payload")
    if frame_type in (CAL_TX_DATA, CAL_TX_DIAG_DATA) and bool(flags & 0x01) != (
        offset + payload_len == total_len
    ):
        raise ValueError("Calibration TX last-fragment flag is inconsistent")
    return CalibrationTxFrame(frame_type, transaction_id, flags, offset, raw[CAL_TX_HEADER_BYTES:], total_len, crc32)


def _record_crc32(raw: bytes) -> int:
    covered = raw[:CAL_RECORD_CRC_OFFSET] + b"\x00\x00\x00\x00" + raw[CAL_RECORD_CRC_OFFSET + 4 :]
    return binascii.crc32(covered) & 0xFFFFFFFF


def parse_calibration_record(data: bytes | bytearray) -> CalibrationRecord:
    raw = bytes(data)
    if not (CAL_RECORD_HEADER_BYTES <= len(raw) <= CAL_RECORD_MAX_BYTES):
        raise ValueError("Calibration record length out of range")
    magic, version, header_bytes, record_bytes, generation, valid_flags, quality, created_unix_ms, crc32 = struct.unpack(
        "<IBBHIIHQI", raw[:CAL_RECORD_HEADER_BYTES]
    )
    if magic != CAL_RECORD_MAGIC or version != CAL_RECORD_VERSION or header_bytes != CAL_RECORD_HEADER_BYTES:
        raise ValueError("Calibration record header is invalid")
    if record_bytes != len(raw):
        raise ValueError("Calibration record byte count mismatch")
    if _record_crc32(raw) != crc32:
        raise ValueError("Calibration record CRC32 mismatch")

    expected_lengths = {1: 48, 2: 48, 3: 36, 4: 48, 5: 36}
    flag_by_type = {
        1: CAL_VALID_IMU_GYRO,
        2: CAL_VALID_IMU_ACCEL,
        3: CAL_VALID_IMU_INSTALL,
        4: CAL_VALID_MAG_MODEL,
        5: CAL_VALID_MAG_INSTALL,
    }
    offset = CAL_RECORD_HEADER_BYTES
    seen_types: set[int] = set()
    present_flags = 0
    tlvs: list[tuple[int, bytes]] = []
    mag_bias: tuple[float, float, float] | None = None
    mag_matrix: tuple[float, ...] | None = None
    while offset < len(raw):
        if len(raw) - offset < 2:
            raise ValueError("Calibration TLV header is truncated")
        tlv_type, tlv_len = raw[offset], raw[offset + 1]
        offset += 2
        if tlv_len == 0 or offset + tlv_len > len(raw):
            raise ValueError("Calibration TLV length is invalid")
        value = raw[offset : offset + tlv_len]
        offset += tlv_len
        if tlv_type in expected_lengths:
            if tlv_type in seen_types or tlv_len != expected_lengths[tlv_type]:
                raise ValueError("Calibration known TLV is duplicated or has invalid length")
            seen_types.add(tlv_type)
            present_flags |= flag_by_type[tlv_type]
            floats = struct.unpack(f"<{tlv_len // 4}f", value)
            if not all(math.isfinite(item) for item in floats):
                raise ValueError("Calibration TLV contains NaN/Inf")
            if tlv_type == CAL_TLV_MAG_MODEL:
                mag_bias = (floats[0], floats[1], floats[2])
                mag_matrix = tuple(floats[3:12])
        tlvs.append((tlv_type, value))
    known_mask = (
        CAL_VALID_IMU_GYRO
        | CAL_VALID_IMU_ACCEL
        | CAL_VALID_IMU_INSTALL
        | CAL_VALID_MAG_MODEL
        | CAL_VALID_MAG_INSTALL
    )
    if valid_flags & known_mask != present_flags:
        raise ValueError("Calibration valid flags do not match TLVs")
    model_flags = valid_flags & CAL_MAG_MODEL_FLAG_MASK
    if model_flags == CAL_MAG_MODEL_FLAG_MASK:
        raise ValueError("Calibration MAG model flags are mutually exclusive")
    if model_flags and not (valid_flags & CAL_VALID_MAG_MODEL):
        raise ValueError("Calibration MAG model grade requires MAG model data")
    return CalibrationRecord(
        generation,
        valid_flags,
        quality,
        created_unix_ms,
        record_bytes,
        crc32,
        tuple(tlvs),
        mag_bias,
        mag_matrix,
        raw,
    )


def parse_mag_calibration_diagnostics(
    data: bytes | bytearray,
) -> MagCalibrationDiagnostics:
    raw = bytes(data)
    if len(raw) != CAL_MAG_DIAG_BYTES:
        raise ValueError(
            f"Mag calibration diagnostics must be {CAL_MAG_DIAG_BYTES} bytes, got {len(raw)}"
        )
    magic = struct.unpack_from("<I", raw, 0)[0]
    if magic != CAL_MAG_DIAG_MAGIC or raw[4] != CAL_MAG_DIAG_VERSION:
        raise ValueError("Mag calibration diagnostics magic/version mismatch")
    if raw[31] != 0 or struct.unpack_from("<H", raw, 34)[0] != 0:
        raise ValueError("Mag calibration diagnostics reserved field is invalid")
    expected_crc = struct.unpack_from("<I", raw, CAL_MAG_DIAG_CRC_OFFSET)[0]
    actual_crc = binascii.crc32(raw[:CAL_MAG_DIAG_CRC_OFFSET]) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError("Mag calibration diagnostics CRC32 mismatch")
    min_values = struct.unpack_from("<III", raw, 40)
    max_values = struct.unpack_from("<III", raw, 52)
    return MagCalibrationDiagnostics(
        transaction_id=raw[5],
        completion_status=raw[6],
        final_model=raw[7],
        sample_count=struct.unpack_from("<I", raw, 8)[0],
        rejected_sample_count=struct.unpack_from("<I", raw, 12)[0],
        sensor_read_errors=struct.unpack_from("<I", raw, 16)[0],
        elapsed_ms=struct.unpack_from("<I", raw, 20)[0],
        fit_attempts=raw[24],
        coverage_mask=raw[25],
        direction_mask=raw[26],
        final_ready_status=raw[27],
        full_solve_status=raw[28],
        axis_solve_status=raw[29],
        hard_solve_status=raw[30],
        quality=struct.unpack_from("<H", raw, 32)[0],
        rms_x1e6=struct.unpack_from("<I", raw, 36)[0],
        min_values=tuple(min_values),
        max_values=tuple(max_values),
        crc32=expected_crc,
        raw=raw,
    )


class CalibrationReceiver:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.transaction_id: int | None = None
        self.total_len = 0
        self.crc32 = 0
        self.transfer_kind = ""
        self._buffer = bytearray()
        self._received = bytearray()
        self.received_count = 0

    def on_notify(self, data: bytes | bytearray) -> CalibrationReceiverEvent:
        frame = parse_calibration_tx_frame(data)
        if frame.frame_type in (CAL_TX_START, CAL_TX_DIAG_START):
            transfer_kind = (
                "diagnostics" if frame.frame_type == CAL_TX_DIAG_START else "record"
            )
            valid_length = (
                frame.total_len == CAL_MAG_DIAG_BYTES
                if transfer_kind == "diagnostics"
                else CAL_RECORD_HEADER_BYTES <= frame.total_len <= CAL_RECORD_MAX_BYTES
            )
            if not valid_length:
                raise ValueError("Calibration START total length is invalid")
            self.transaction_id = frame.transaction_id
            self.total_len = frame.total_len
            self.crc32 = frame.crc32
            self.transfer_kind = transfer_kind
            self._buffer = bytearray(frame.total_len)
            self._received = bytearray(frame.total_len)
            self.received_count = 0
            return CalibrationReceiverEvent(
                "start", frame.transaction_id, 0, frame.total_len,
                transfer_kind=transfer_kind,
            )

        if self.transaction_id is None:
            raise ValueError("Calibration DATA/END received before START")
        if (
            frame.transaction_id != self.transaction_id
            or frame.total_len != self.total_len
            or frame.crc32 != self.crc32
        ):
            raise ValueError("Calibration TX transaction metadata changed")
        expected_data = (
            CAL_TX_DIAG_DATA if self.transfer_kind == "diagnostics" else CAL_TX_DATA
        )
        expected_end = (
            CAL_TX_DIAG_END if self.transfer_kind == "diagnostics" else CAL_TX_END
        )
        if frame.frame_type not in (expected_data, expected_end):
            raise ValueError("Calibration TX frame family changed")

        if frame.frame_type == expected_data:
            for index, value in enumerate(frame.payload, start=frame.offset):
                if self._received[index] and self._buffer[index] != value:
                    raise ValueError("Conflicting duplicate Calibration DATA byte")
                if not self._received[index]:
                    self._received[index] = 1
                    self.received_count += 1
                self._buffer[index] = value
            return CalibrationReceiverEvent(
                "data", frame.transaction_id, self.received_count, self.total_len,
                transfer_kind=self.transfer_kind,
            )

        if frame.offset != self.total_len or self.received_count != self.total_len:
            raise ValueError("Calibration END arrived before all bytes were received")
        if self.transfer_kind == "diagnostics":
            diagnostics = parse_mag_calibration_diagnostics(bytes(self._buffer))
            if diagnostics.crc32 != self.crc32:
                raise ValueError("Calibration TX CRC does not match diagnostics CRC")
            event = CalibrationReceiverEvent(
                "complete",
                frame.transaction_id,
                self.received_count,
                self.total_len,
                diagnostics=diagnostics,
                transfer_kind="diagnostics",
            )
        else:
            record = parse_calibration_record(bytes(self._buffer))
            if record.crc32 != self.crc32:
                raise ValueError("Calibration TX CRC does not match record CRC")
            event = CalibrationReceiverEvent(
                "complete",
                frame.transaction_id,
                self.received_count,
                self.total_len,
                record=record,
                transfer_kind="record",
            )
        self.reset()
        return event
