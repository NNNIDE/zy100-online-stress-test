"""Strict, read-only decoder for STRESS_IO v1 and optional Host timing evidence."""
from pathlib import Path
import re

SCHEMA = ["fifo flags", "calls total", "peak unit", "calls total", "peak unit",
          "calls total", "peak unit", "calls total", "peak unit", "bus wip",
          "spans total", "span_max poll_max", "open_us pending", "bus_us wip_us", "end lines"]
IDENTITY = ("v", "fw", "sid", "start")


def parse_io(text: str, identity: dict, required: bool) -> dict | None:
    rows = []
    for line in text.splitlines():
        if "[STRESS_IO]" not in line:
            continue
        payload = line.split("[STRESS_IO]", 1)[1].strip()
        pairs = re.findall(r"(\w+)=([0-9]+)", payload)
        fields = dict(pairs)
        if len(pairs) != len(fields) or "\x00" in payload or set(IDENTITY) - fields.keys():
            raise ValueError("STRESS_IO身份缺失、字段重复或截断")
        if any(int(fields[k]) != identity[k] for k in ("fw", "sid", "start")):
            continue
        if re.sub(r"\w+=[0-9]+", "", payload).strip():
            raise ValueError("STRESS_IO含无效字段")
        rows.append({k: int(v) for k, v in pairs})
    if not rows and not required:
        return None
    if len(rows) != 15 or [r.get("n") for r in rows] != list(range(15)):
        raise ValueError("STRESS_IO缺行、重复、乱序或场次不唯一")
    for i, r in enumerate(rows):
        if set(r) != set(IDENTITY) | {"n"} | set(SCHEMA[i].split()):
            raise ValueError("STRESS_IO字段不匹配")
        if r["v"] != 1 or any(v < 0 or v > 0xFFFFFFFF for v in r.values()):
            raise ValueError("STRESS_IO版本或数值不支持")
    if rows[14]["end"] != 1 or rows[14]["lines"] != 15 or rows[0]["fifo"] not in (0, 1):
        raise ValueError("STRESS_IO结束标记或开关错误")
    if rows[0]["flags"] & ~1:
        raise ValueError("STRESS_IO计数溢出、时钟不可用或未知状态")
    if rows[12]["pending"] != rows[0]["flags"] or (not rows[12]["pending"] and rows[12]["open_us"]):
        raise ValueError("STRESS_IO未完成页状态不一致")
    metrics = {}
    for i, name in enumerate(("program", "readback", "transmit_read", "status_read")):
        a, b = rows[1 + i * 2], rows[2 + i * 2]
        if b["unit"] != 1 or b["peak"] > a["total"] or (a["calls"] == 0 and a["total"]):
            raise ValueError("STRESS_IO耗时口径错误")
        metrics[name] = dict(calls=a["calls"], total_us=a["total"], peak_us=b["peak"])
    if rows[10]["spans"] > metrics["program"]["calls"] or rows[11]["span_max"] > rows[10]["total"]:
        raise ValueError("STRESS_IO完成观察计数错误")
    return dict(identity={k: rows[0][k] for k in IDENTITY}, fifo=rows[0]["fifo"], metrics=metrics,
                bus_busy_count=rows[9]["bus"], wip_busy_count=rows[9]["wip"],
                observed_completion_count=rows[10]["spans"], observed_completion_total_us=rows[10]["total"],
                observed_completion_max_us=rows[11]["span_max"], check_gap_max_us=rows[11]["poll_max"],
                pending=rows[12]["pending"], open_age_us=rows[12]["open_us"],
                after_bus_observation_us=rows[13]["bus_us"], after_wip_observation_us=rows[13]["wip_us"],
                note="调用耗时含抢占；完成时间为首次观察上界；BUSY后间隔含调度，均不是CPU占用或纯硬件忙时间")


def host_timing(path: Path | None, session: Path, sid: int) -> dict:
    if path is None:
        return {"evidence": "证据不足", "reason": "未提供Host日志，不能判断保存与ACK耗时"}
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    starts = [i for i, line in enumerate(lines) if "[ONLINE_STATE] start " in line and session.name in line]
    if len(starts) != 1:
        return {"evidence": "证据不足", "reason": "Host会话目录匹配不唯一"}
    candidates = []
    for line in lines[starts[0] + 1:]:
        if "[ONLINE_STATE] start " in line:
            break
        if "[ONLINE_HOST_FINAL]" in line and re.search(rf"\bsid={sid}\b", line):
            candidates.append(line)
    if len(candidates) != 1:
        return {"evidence": "证据不足", "reason": "Host结束统计缺失或重复"}
    line = candidates[0]
    values = dict(re.findall(r"\b(\w+)=([^\s]+)", line))
    keys = ("cb", "bytes", "q", "save", "ack", "saved_to_ack", "storage", "all_qmax",
            "all_save", "all_queue", "all_storage_work", "all_saved_to_ack", "percentiles")
    if any(k not in values for k in ("bytes", "q", "save", "ack", "saved_to_ack")):
        return {"evidence": "证据不足", "reason": "Host结束统计字段不完整", "raw_line": line}
    return {"evidence": "完整", "fields": {k: values[k] for k in keys if k in values}, "raw_line": line,
            "note": "保留Host原字段单位/分位数口径；累计字节不是模拟净负载，ACK统计不是空中链路单向时延"}
