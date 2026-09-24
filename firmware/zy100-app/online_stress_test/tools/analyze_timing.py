"""Join immutable Host files and versioned UART timing evidence (no device I/O)."""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta
import hashlib
import json
from pathlib import Path
import re
import sys
from stress_io_report import parse_io, host_timing

STAGES = ("其他/调度间隔", "RAW写入", "RAW提交", "擦除", "空间预留等待",
          "STRESS页编程", "STRESS读回", "STRESS提交", "等待完整数据块")
IDENTITY = ("v", "fw", "sid", "start")
FIELDS_V1 = {
    "begin": "rate elapsed error water released pump_max prep_erases transitions events",
    "gap": "from to water_from water_to freed open",
    "record": "type age page pages raw_max stress_max",
    "stage": "stage total peak gap", "event": "ms stage water", "end": "lines",
}
FIELDS_V2 = {
    "begin": "rate elapsed", "state": "error water", "count": "released pump_max",
    "prep": "prep_erases events", "trace": "transitions", "gap": "from to",
    "water": "water_from water_to", "release": "freed open", "record": "type age",
    "page": "page pages", "max": "raw stress", "stage": "stage total peak",
    "part": "stage gap", "event": "ms stage water", "end": "lines",
}


def parse_groups(text: str) -> list[dict]:
    groups: list[dict] = []
    current = None
    for line in text.splitlines():
        if "[STRESS_DIAG]" not in line:
            continue
        payload = line.split("[STRESS_DIAG]", 1)[1]
        pairs = re.findall(r"(\w+)=(\w+)", payload)
        values = dict(pairs)
        if len(values) != len(pairs):
            values["malformed"] = "duplicate_key"
        if "\x00" in payload:
            values["malformed"] = "truncated_or_nul"
        if values.get("kind") == "begin" or current is None:
            current = {"rows": [], "wall_line": line}
            groups.append(current)
        current["rows"].append(values)
    return groups


def validate_group(group: dict) -> dict:
    rows = group["rows"]
    for row in rows:
        if "malformed" in row:
            raise ValueError("诊断行被截断、含NUL或字段重复")
    ident = {k: int(rows[0][k]) for k in IDENTITY}
    if ident["v"] not in (1, 2):
        raise ValueError("不支持的诊断版本")
    schema = FIELDS_V1 if ident["v"] == 1 else FIELDS_V2
    for row in rows:
        kind = row.get("kind")
        if kind not in schema or set(row) != set(IDENTITY) | {"n", "kind"} | set(schema[kind].split()):
            raise ValueError("诊断行字段缺失或未知")
        if any(not re.fullmatch(r"[0-9]+", v) or int(v) > 0xFFFFFFFF
               for k, v in row.items() if k != "kind"):
            raise ValueError("诊断数值无效")
    if any({k: int(row[k]) for k in IDENTITY} != ident for row in rows):
        raise ValueError("诊断行身份不一致")
    if [int(row["n"]) for row in rows] != list(range(len(rows))):
        raise ValueError("诊断行缺失、重复或乱序")
    kinds = [row["kind"] for row in rows]
    if kinds[0] != "begin" or kinds[-1] != "end":
        raise ValueError("诊断头部或结束标记缺失")
    n_events = int(rows[0 if ident["v"] == 1 else 3]["events"])
    expected = (["begin", "gap", "record"] + ["stage"] * 9 if ident["v"] == 1 else
                ["begin", "state", "count", "prep", "trace", "gap", "water", "release", "record", "page", "max"]
                + ["stage", "part"] * 9)
    if not 0 <= n_events <= 12 or kinds != expected + ["event"] * n_events + ["end"]:
        raise ValueError("诊断行结构不完整")
    if int(rows[-1]["lines"]) != len(rows):
        raise ValueError("诊断结束行数不匹配")
    numeric = [{k: int(v) for k, v in row.items() if k != "kind"} for row in rows]
    if ident["v"] == 1:
        begin, gap, record = numeric[:3]
        stages, events = numeric[3:12], numeric[12:-1]
    else:
        def merge(first: int, last: int) -> dict:
            combined = dict(numeric[first])
            for i in range(first + 1, last):
                combined.update({k: v for k, v in numeric[i].items() if k not in (*IDENTITY, "n")})
            return combined
        begin, gap, record = merge(0, 5), merge(5, 8), merge(8, 10)
        record.update(raw_max=numeric[10]["raw"], stress_max=numeric[10]["stress"])
        stages = []
        for i in range(11, 29, 2):
            if numeric[i]["stage"] != numeric[i+1]["stage"]:
                raise ValueError("阶段分行编号不匹配")
            stages.append(merge(i, i+2))
        events = numeric[29:-1]
    if [s["stage"] for s in stages] != list(range(9)):
        raise ValueError("阶段编号缺失或重复")
    if sum(s["total"] for s in stages) != begin["elapsed"]:
        raise ValueError("阶段时间总和不等于诊断时长")
    if not 0 <= gap["from"] <= gap["to"] <= begin["elapsed"]:
        raise ValueError("无释放区间越界")
    if sum(s["gap"] for s in stages) != gap["to"] - gap["from"]:
        raise ValueError("无释放区间的阶段归因不完整")
    if any(s["peak"] > s["total"] or s["gap"] > s["total"] for s in stages):
        raise ValueError("阶段时间越界")
    if (max(begin["water"], gap["water_from"], gap["water_to"]) > 4096
            or record["page"] > record["pages"] or record["type"] not in (0, 1, 4)):
        raise ValueError("缓存或记录进度越界")
    if any(e["stage"] >= 9 or e["water"] > 4096 or e["ms"] > begin["elapsed"] for e in events):
        raise ValueError("事件字段越界")
    if [e["ms"] for e in events] != sorted(e["ms"] for e in events):
        raise ValueError("事件顺序错误")
    return {"identity": ident, "summary": begin, "gap": gap, "record": record,
            "stages": stages, "events": events}


def wall_time_matches(line: str, metadata: dict, elapsed_ms: int) -> bool:
    """Zero-load records carry no source start tick; require PC wall-time evidence."""
    try:
        start = datetime.strptime(metadata["host_started_at"], "%Y-%m-%dT%H:%M:%S%z")
        m = re.search(r"(\d{2})-(\d{2})#(\d{2}:\d{2}:\d{2}(?:\.\d+)?)", line)
        if not m:
            return False
        clock = datetime.fromisoformat(f"{start.year}-{m[1]}-{m[2]}T{m[3]}").replace(tzinfo=start.tzinfo)
        expected = start + timedelta(milliseconds=elapsed_ms)
        # PC UART display timestamp versus PC Host receipt; not an MCU clock sync claim.
        return abs((clock - expected).total_seconds()) <= 5
    except (KeyError, ValueError):
        return False


def analyze(folder: Path, uart: Path, host_root: Path, host_log: Path | None = None) -> dict:
    result = {"schema_version": 1, "evidence": "证据不足", "issues": [],
              "session_directory": str(folder), "uart_log": str(uart),
              "scope": "固定4 KiB条件下的观察阶段停留时间；不是CPU使用率或BLE极限"}
    try:
        sys.path.insert(0, str(host_root))
        from ble.online_stress_report import build_stress_report
        report_path = folder / "stress_report.json"
        old = json.loads(report_path.read_text(encoding="utf-8-sig"))
        metadata = json.loads((folder / "metadata.json").read_text(encoding="utf-8-sig"))
        report = build_stress_report(folder, metadata, old["samples"], old["terminal"])
        keys = ("file_sha256", "disk_record_counts", "disk_stress_bytes", "disk_stress_blocks", "disk_imu_packets", "disk_mag_samples")
        if any(report[k] != old[k] for k in keys):
            raise ValueError("落盘复核与原报告不一致")
        if any(".bin:" in item for item in report["failures"]):
            raise ValueError("落盘记录校验失败")
        result["original_verdict"] = old["verdict"]
        result["file_sha256"] = report["file_sha256"]
        result["input_sha256"] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in (report_path, folder / "metadata.json", uart)}
        config, terminal = old["configuration"], old["terminal"]
        fw = int(re.search(r"code=(\d+)", config["firmware"])[1])
        starts = set()
        for name in report["file_sha256"]:
            h = json.loads((folder / "headers" / (Path(name).stem + ".json")).read_text(encoding="utf-8"))
            if h["record"].get("stress"):
                starts.add(h["record"]["stress"]["start_ms"])
        if len(starts) > 1:
            raise ValueError("落盘记录存在多个源开始时刻")
        candidates = []
        for group in parse_groups(uart.read_text(encoding="utf-8-sig", errors="replace")):
            row = group["rows"][0]
            if (row.get("fw") == str(fw) and row.get("sid") == str(old["session_id"])
                    and row.get("rate") == str(config["rate_Bps"])):
                if starts and int(row.get("start", -1)) not in starts:
                    continue
                if not starts and not wall_time_matches(group["wall_line"], metadata, int(row.get("elapsed", 0))):
                    continue
                candidates.append(group)
        if len(candidates) != 1:
            raise ValueError(f"场次匹配不唯一或缺少身份证据：候选数={len(candidates)}")
        diag = validate_group(candidates[0])
        if terminal is None:
            raise ValueError("缺少设备终态")
        if diag["summary"]["error"] == 1 and (terminal["failure_code"] != 1 or diag["summary"]["water"] != terminal["ring_bytes"]):
            raise ValueError("诊断首错/水位与设备终态不一致")
        result["io"] = parse_io(uart.read_text(encoding="utf-8-sig"), diag["identity"], fw >= 20006)
        result["host_timing"] = host_timing(host_log, folder, terminal["session_id"])
        result["frozen_pending_bytes"] = terminal.get("pending_bytes")
        result["cleanup_note"] = "异常清理后的pending=0不代表冻结时积压已确认"
        slots = terminal["elapsed_ms"] // 10  # Existing MAG scheduler: 10000 us.
        result["mag_timing"] = dict(
            missed_deadlines=terminal["mag_missed_deadlines"], nominal_slots=slots,
            missed_slot_percent=(100 * terminal["mag_missed_deadlines"] / slots if slots else None),
            note="调度超期次数/本场名义100 Hz期限数，不是物理丢样率；允许少量miss，无自动豁免阈值")
        result.update(evidence="完整", diagnosis=diag)
        result["observed_rates_Bps"] = {
            "generated": terminal["generated_bytes"] * 1000 / max(1, terminal["elapsed_ms"]),
            "verified_release": diag["summary"]["released"] * 1000 / max(1, diag["summary"]["elapsed"]),
            "note": "本场观测速率；分母分别为产生时长和写入诊断时长，不是稳态能力保证",
        }
        gap = diag["gap"]["to"] - diag["gap"]["from"]
        ranked = sorted(diag["stages"], key=lambda s: s["gap"], reverse=True)
        known = [s for s in ranked if s["stage"] not in (0, 8)]
        lead = known[0]
        # Majority is a descriptive classification, not proof of a hardware bottleneck.
        if config["rate_Bps"] == 0:
            result["finding"] = "零新增负载没有模拟缓存释放区间；仅作数据与时序对照，不据此定位新增负载瓶颈"
        elif not gap or lead["gap"] * 2 <= gap:
            result["finding"] = "没有单一已知阶段占最长停写区间的一半以上，需结合混合阶段或调度间隔继续定位"
        else:
            result["finding"] = f"最长未释放区间 {gap} ms，观察到 {STAGES[lead['stage']]} 占 {lead['gap']} ms；这是优先排查方向"
        result["limits"] = ["阶段停留包含调度间隔，不等于Flash芯片忙时间",
                            "短失败场不能区分稳态平均吞吐极限与偶发停顿，不从缓存峰值推算CPU占用率",
                            "最近12次事件可能不覆盖最长区间；该区间的分阶段累计独立保留"]
        if fw < 20003:
            result["limits"].append("旧固件erase_count为累计口径，不能证明本场擦除覆盖")
    except (OSError, ValueError, KeyError, TypeError, ImportError, IndexError) as exc:
        result["issues"].append(str(exc))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", required=True, type=Path)
    parser.add_argument("--uart-log", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--host-root", type=Path, default=Path("D:/BLE_Tool"))
    parser.add_argument("--host-log", type=Path, help="可选：匹配本场的Host日志")
    args = parser.parse_args()
    folder, out = args.session_dir.resolve(), args.output_dir.resolve()
    if out == folder or folder in out.parents:
        parser.error("输出必须位于原会话目录之外")
    result = analyze(folder, args.uart_log.resolve(), args.host_root.resolve(), args.host_log)
    out.mkdir(parents=True, exist_ok=True)
    (out / "timing_diagnosis.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = ["# 压测写入等待定位", "", f"证据：{result['evidence']}",
             f"原测试结论：{result.get('original_verdict', '未知')}", "", result.get("finding", "尚不能定位"), ""]
    if "diagnosis" in result:
        d = result["diagnosis"]
        lines += [f"失败/结束水位：{d['summary']['water']} / 4096 B；压测源pump调用最大间隔：{d['summary']['pump_max']} ms。",
                  f"最长区间：{d['gap']}。", f"未完成记录及各类记录最大耗时：{d['record']}。", "",
                  "| 阶段 | 总停留ms | 最长连续ms | 最长未释放区间内ms |", "|---|---:|---:|---:|"]
        lines += [f"| {STAGES[s['stage']]} | {s['total']} | {s['peak']} | {s['gap']} |" for s in d['stages']]
    if result.get("io"):
        io = result["io"]
        lines += ["", "| I/O调用 | 次数 | 累计us | 最大us |", "|---|---:|---:|---:|"]
        lines += [f"| {name} | {m['calls']} | {m['total_us']} | {m['peak_us']} |" for name, m in io["metrics"].items()]
        lines += ["", f"完成观察上界最大 {io['observed_completion_max_us']} us；检查间隔最大 {io['check_gap_max_us']} us；未完成页 {io['pending']}，已用 {io['open_age_us']} us。", io["note"]]
    if "host_timing" in result:
        lines += ["", "Host时序：" + json.dumps(result["host_timing"], ensure_ascii=False),
                  f"冻结待传：{result.get('frozen_pending_bytes')} B；异常清理为零不代表全部确认。"]
    if "mag_timing" in result:
        lines += ["", "MAG时序：" + json.dumps(result["mag_timing"], ensure_ascii=False)]
    lines += ["", *["- " + x for x in result["issues"] + result.get("limits", [])]]
    (out / "timing_diagnosis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(result["evidence"] + ": " + result.get("finding", "; ".join(result["issues"])))
    return 0 if result["evidence"] == "完整" else 2


if __name__ == "__main__":
    raise SystemExit(main())
