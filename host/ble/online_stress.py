"""ZY100 diagnostic v1 wire contract and file-based integrity assessment."""
from __future__ import annotations

import struct
from dataclasses import asdict, dataclass

CMD_STRESS = 0x30
CAP_STRESS = 0x20
RECORD_STRESS = 4
FRAME_STRESS = 0x15
RATES = (0, 16000, 32000, 48000, 64000, 96000)
PERIOD_MS = 20
RING_BYTES = 4096
QUERY_DETAIL = 0x010C143F
HEADER = struct.Struct("<4sHH14I")
STATUS = struct.Struct("<HH40I")
STATUS_NAMES = (
    "session_id rate_Bps elapsed_ms expected_bytes generated_bytes rejected_bytes "
    "committed_bytes acked_bytes ring_bytes ring_peak_bytes generation_delay_max_ms "
    "cache_wait_count flash_wait_count send_wait_count ack_wait_count failure_code "
    "stop_reason pending_bytes pending_peak_bytes ack_max_ms erase_count wrap_count "
    "status_skipped imu_packets mag_samples mag_missed_deadlines mag_lateness_max_us "
    "mag_interval_max_us page_build_max_us page_program_max_us page_verify_max_us "
    "record_write_max_ms heap_data_min_bytes heap_buffer_min_bytes app_stack_min_bytes "
    "worker_stack_min_bytes worker_gap_max_ms mtu connection_interval_units phy"
).split()
FAILURES = {
    0: "无", 1: "4 KiB 模拟数据缓存不足", 2: "计数或时间超出范围",
    3: "Flash reservation 失败", 4: "Flash 写入失败", 5: "Flash 读回校验失败",
    6: "Flash 提交失败", 7: "工作区或缓存所有权错误",
    0x102: "BLE 断连", 0x103: "ACK 超时", 0x104: "Flash Spool 已满",
    0x105: "内部错误", 0x106: "传输不可用", 0x107: "发送停滞",
    0x108: "发送完成失败", 0x109: "采集致命错误", 0x10A: "低电量",
    0x10B: "关机", 0x10C: "Flash I/O 失败", 0x10D: "复位恢复",
}


@dataclass(frozen=True, slots=True)
class StressRecord:
    session_id: int
    rate_Bps: int
    period_ms: int
    first_block: int
    block_count: int
    byte_offset: int
    data_bytes: int
    planned_ms: int
    first_delay_ms: int
    second_delay_ms: int
    pattern_mode: int
    start_ms: int

    def to_dict(self) -> dict:
        return asdict(self)


def pattern(seed: int, offset: int, length: int) -> bytes:
    return bytes(((x := (offset + i) ^ seed) ^ (x >> 8) ^ (x >> 16) ^ (x >> 24)) & 255
                 for i in range(length))


def parse_stress_source(source: bytes, session_id: int) -> StressRecord:
    if len(source) < HEADER.size or len(source) % 256:
        raise ValueError("STRESS source length/alignment")
    magic, version, size, *v = HEADER.unpack_from(source)
    if magic != b"STRS" or version != 1 or size != HEADER.size or v[12:] != [0, 0]:
        raise ValueError("STRESS header/version/reserved")
    r = StressRecord(*v[:12])
    if r.session_id != session_id or r.rate_Bps not in RATES[1:] or r.period_ms != PERIOD_MS:
        raise ValueError("STRESS session/rate/period")
    block_bytes = r.rate_Bps // 50
    if (r.block_count not in (1, 2) or r.byte_offset != r.first_block * block_bytes
            or r.data_bytes != r.block_count * block_bytes or r.pattern_mode != 1
            or r.planned_ms != (r.first_block + 1) * PERIOD_MS
            or (r.block_count == 1 and r.second_delay_ms != 0)):
        raise ValueError("STRESS block/byte range or pattern mode")
    end = HEADER.size + r.data_bytes
    if len(source) != (end + 255) // 256 * 256:
        raise ValueError("STRESS padded length")
    if source[HEADER.size:end] != pattern(r.session_id, r.byte_offset, r.data_bytes):
        raise ValueError("STRESS deterministic content mismatch")
    if source[end:] != b"\xff" * (len(source) - end):
        raise ValueError("STRESS padding mismatch")
    return r


def parse_stress_status(payload: bytes, session_id: int | None = None) -> dict:
    if len(payload) != STATUS.size:
        raise ValueError("STRESS status length")
    version, size, *values = STATUS.unpack(payload)
    if version != 1 or size != STATUS.size:
        raise ValueError("STRESS status version")
    result = dict(zip(STATUS_NAMES, values))
    if (session_id is not None and result["session_id"] != session_id
            or result["rate_Bps"] not in RATES
            or result["ring_bytes"] > RING_BYTES
            or result["ring_peak_bytes"] > RING_BYTES):
        raise ValueError("STRESS status session/rate/ring")
    return result


def assess_backlog(samples: list[dict], max_record_bytes: int) -> tuple[bool, bool]:
    """Return (unsustainable, complete); do not interpolate missing telemetry."""
    windows: dict[int, list[dict]] = {}
    for s in samples:
        ms = s["elapsed_ms"]
        if ms >= 30000:
            windows.setdefault((ms - 30000) // 30000, []).append(s)
    minima = {}
    complete = bool(samples)
    previous = None
    for s in sorted(samples, key=lambda v: v["elapsed_ms"]):
        if previous is not None and s["elapsed_ms"] - previous > 2500:
            complete = False
        previous = s["elapsed_ms"]
    for window, rows in windows.items():
        times = {r["elapsed_ms"] // 1000 for r in rows}
        if len(times) >= 28:
            minima[window] = min(r["pending_bytes"] for r in rows)
    for i in sorted(minima):
        if i + 1 in minima and i + 2 in minima:
            a, b, c = (minima[i + j] for j in range(3))
            if a < b < c and c - a >= max_record_bytes:
                return True, complete
    return False, complete
