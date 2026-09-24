"""Re-open persisted BINs; a telemetry counter is never proof of integrity."""
from __future__ import annotations

import json
from pathlib import Path

from .online_stress import FAILURES, RECORD_STRESS, assess_backlog
from .zy100_protocol import (
    OnlineRecordFragment, parse_online_spool_header, validate_online_completed_record,
)


def build_stress_report(folder: Path, metadata: dict, samples: list[dict], terminal: dict | None) -> dict:
    config = metadata.get("stress_config") or {}
    failures: list[str] = []
    review: list[str] = []
    counts = {1: 0, 2: 0, 3: 0, 4: 0}
    total = blocks = imu = mag = 0
    max_record = 1
    file_hashes = {}
    import hashlib
    for path in sorted((folder / "samples").glob("*.bin"), key=lambda p: int(p.stem.split("_")[-1])):
        try:
            data = path.read_bytes()
            h = parse_online_spool_header(data)
            if h.session_id != metadata["session_id"]:
                raise ValueError("wrong session")
            fragment = OnlineRecordFragment(h.session_id, h.record_type, 1, h.record_id,
                                            0, len(data), len(data), h.crc32, data, 0)
            record = validate_online_completed_record(fragment, data)
            counts[h.record_type] += 1
            max_record = max(max_record, len(data))
            # Header persistence is part of the same save-and-ACK contract.
            header_path = folder / "headers" / (path.stem + ".json")
            if not header_path.is_file():
                raise ValueError("missing saved Header JSON")
            saved_header = json.loads(header_path.read_text(encoding="utf-8"))
            if (saved_header.get("online_spool_header_hex") != data[:256].hex()
                    or saved_header.get("record") != record.to_dict()):
                raise ValueError("saved Header JSON differs from BIN")
            file_hashes[path.name] = hashlib.sha256(data).hexdigest()
            if record.continuous_raw is not None:
                imu += record.continuous_raw.packet_count
            if record.mag_raw is not None:
                mag += record.mag_raw.sample_count
            if record.stress is not None:
                r = record.stress
                if r.first_block != blocks or r.byte_offset != total or r.rate_Bps != config.get("rate_Bps"):
                    raise ValueError("missing/duplicate/out-of-order STRESS block or wrong rate")
                blocks += r.block_count
                total += r.data_bytes
        except (OSError, ValueError, KeyError, TypeError) as exc:
            failures.append(f"{path.name}: {exc}")
    end = metadata.get("end") or {}
    if metadata.get("mag_read_error_count", 0):
        failures.append("MAG 采集读取错误")
    if metadata.get("packet_sequence_continuous") is False:
        failures.append("IMU 采集序号不连续")
    if metadata.get("status") != "ended":
        failures.append(metadata.get("last_error") or "本场异常终止或未确认 END")
    if terminal is None:
        review.append("缺少设备终态统计")
    else:
        if terminal["rate_Bps"] != config.get("rate_Bps"):
            failures.append("设备终态档位与本场配置不一致")
        if terminal["stop_reason"] != 1:
            failures.append(f"非正常停止：stop_reason={terminal['stop_reason']}")
        if terminal["elapsed_ms"] + 1000 < int(config.get("duration_s", 0)) * 1000:
            review.append("采集时长未达到设定值，本场不能代替该时长的稳定性验证")
        expected = terminal["elapsed_ms"] // 20 * (terminal["rate_Bps"] // 50)
        values = [expected, terminal["expected_bytes"], terminal["generated_bytes"],
                  terminal["committed_bytes"], total, terminal["acked_bytes"]]
        if len(set(values)) != 1:
            failures.append(f"期望/设备期望/生成/提交/落盘/设备确认数量不一致：{values}")
        if terminal["failure_code"]:
            failures.append(FAILURES.get(terminal["failure_code"], f"错误 {terminal['failure_code']}"))
        if terminal["rejected_bytes"] or terminal["pending_bytes"] or terminal["ring_bytes"]:
            failures.append("终态仍有拒收、Flash 待传或未提交缓存")
        if imu != terminal["imu_packets"] or mag != terminal["mag_samples"]:
            failures.append("IMU／MAG 采集数量与落盘数量不一致")
        for key in ("app_stack_min_bytes", "worker_stack_min_bytes", "worker_gap_max_ms"):
            if terminal[key] == 0xFFFFFFFF:
                review.append(f"未测得 {key}")
        if not samples or samples[0]["elapsed_ms"] > 2500 or terminal["elapsed_ms"] - samples[-1]["elapsed_ms"] > 2500:
            review.append("状态曲线首尾不完整")
        if terminal["erase_count"] < 2:
            review.append("未覆盖多次 Flash 擦除回收")
    for kind, name in ((1, "raw"), (2, "summary"), (3, "event")):
        if end and (counts[kind] != end[f"produced_{name}"] or counts[kind] != end[f"acked_{name}"]):
            failures.append(f"{name} 产生、文件和设备确认数不一致")
    if end and counts[4] != int((metadata.get("ack_counts") or {}).get("stress", 0)):
        failures.append("STRESS 文件数与 Host ACK 数不一致")
    unsustainable, complete = assess_backlog(samples, max_record)
    if unsustainable:
        failures.append("连续三个 30 秒窗口最低积压递增，累计超过一条最大记录：负载不可持续")
    if not complete:
        review.append("状态采样有缺口，积压趋势需复核")
    baseline = None
    if config.get("rate_Bps", 0) != 0 and terminal:
        for candidate in sorted(folder.parent.glob("*/stress_report.json"), reverse=True):
            if candidate.parent == folder:
                continue
            try:
                report = json.loads(candidate.read_text(encoding="utf-8"))
                c = report.get("configuration", {})
                if (c.get("rate_Bps") == 0 and report.get("verdict") == "通过"
                        and all(c.get(k) == config.get(k) for k in ("firmware", "host_version", "device_address"))):
                    baseline = report
                    break
            except (OSError, ValueError):
                continue
        if baseline is None:
            review.append("没有同设备、固件及 Host 版本的零负载对照")
        else:
            for key in ("mag_lateness_max_us", "mag_interval_max_us", "worker_gap_max_ms"):
                if terminal[key] > baseline["terminal"][key]:
                    review.append(f"{key} 超出零负载对照：{terminal[key]} > {baseline['terminal'][key]}")
            b = baseline["terminal"]
            if terminal["mag_missed_deadlines"] * max(1, b["elapsed_ms"]) > b["mag_missed_deadlines"] * max(1, terminal["elapsed_ms"]):
                review.append("MAG 错过采样期限的频率超出零负载对照")
    return {
        "schema_version": 1, "session_id": metadata.get("session_id"),
        "verdict": "失败" if failures else ("需复核" if review else "通过"),
        "configuration": config, "terminal": terminal, "failures": failures, "review": review,
        "disk_stress_bytes": total, "disk_stress_blocks": blocks, "disk_record_counts": {str(k): v for k, v in counts.items()},
        "disk_imu_packets": imu, "disk_mag_samples": mag, "max_record_bytes": max_record,
        "unsustainable_backlog": unsustainable, "samples": samples, "file_sha256": file_hashes,
        "scope": "本次 4 KiB 缓存、采样分块、Flash、BLE、设备及 Host 组合；不代表 BLE 或 ZY200 音频极限",
        "baseline_session_id": baseline.get("session_id") if baseline else None,
    }


def report_text(report: dict) -> str:
    lines = [f"链路压测：{report['verdict']}", f"会话：{report['session_id']}",
             f"新增目标速率：{report['configuration'].get('rate_Bps')} B/s",
             f"落盘复核：{report['disk_stress_bytes']} B / {report['disk_stress_blocks']} 块",
             report["scope"]]
    lines += [f"失败：{x}" for x in report["failures"]]
    lines += [f"需复核：{x}" for x in report["review"]]
    lines.append("设备终态（字节、毫秒、微秒单位见字段；不推算 MCU CPU 使用率）：")
    lines.append(json.dumps(report["terminal"], ensure_ascii=False, indent=2))
    return "\n".join(lines) + "\n"
