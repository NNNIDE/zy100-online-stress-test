from __future__ import annotations

import os
import tkinter as tk
from tkinter import ttk, messagebox

from ble.online_stress import FAILURES, RATES


class OnlineStressView(ttk.Frame):
    def __init__(self, parent, manager, identity, host_version):
        super().__init__(parent, padding=16)
        self.manager, self.identity, self.host_version = manager, identity, host_version
        self.available = False
        self.rate = tk.StringVar(value="32000")
        self.duration = tk.StringVar(value="600")
        self.message = tk.StringVar(value="连接独立压测固件，等待 Business Ready 后查询能力。")
        self.metrics = tk.StringVar(value="目标 / 实际产生 / 接收：—\n缓存 / Flash 待传 / ACK 延迟：—")
        self.busy = False
        self.report = ""
        self.points = []
        ttk.Label(self, text="链路压测", font=("Microsoft YaHei UI", 16, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(self, text="IMU／地磁 + 模拟数据 → 4 KiB 缓存 → Flash → BLE → 保存校验 → ACK").grid(row=1, column=0, sticky="w", pady=(6, 16))
        controls = ttk.Frame(self)
        controls.grid(row=2, column=0, sticky="ew")
        ttk.Label(controls, text="新增负载 B/s").grid(row=0, column=0, sticky="w")
        self.rate_box = ttk.Combobox(controls, width=12, state="readonly", textvariable=self.rate, values=RATES)
        self.rate_box.grid(row=0, column=1, padx=(8, 20))
        ttk.Label(controls, text="时长 秒").grid(row=0, column=2)
        self.duration_box = ttk.Spinbox(controls, from_=1, to=3600, width=8, textvariable=self.duration)
        self.duration_box.grid(row=0, column=3, padx=(8, 20))
        self.query_button = ttk.Button(controls, text="查询能力", command=self.probe)
        self.query_button.grid(row=0, column=4, padx=4)
        self.start_button = ttk.Button(controls, text="开始压测", command=self.start, state="disabled")
        self.start_button.grid(row=0, column=5, padx=4)
        self.stop_button = ttk.Button(controls, text="停止并收尾", command=manager.stop_stress, state="disabled")
        self.stop_button.grid(row=0, column=6, padx=4)
        ttk.Label(self, textvariable=self.message, wraplength=950).grid(row=3, column=0, sticky="w", pady=(16, 10))
        ttk.Label(self, textvariable=self.metrics, font=("Microsoft YaHei UI", 11), justify="left").grid(row=4, column=0, sticky="w", pady=10)
        ttk.Label(self, text="Flash 待传量趋势（最近 300 次设备状态；曲线缺口会标记需复核）").grid(row=5, column=0, sticky="w")
        self.chart = tk.Canvas(self, height=200, background="#f6f8fb", highlightthickness=0)
        self.chart.grid(row=6, column=0, sticky="nsew", pady=8)
        self.open_button = ttk.Button(self, text="打开本场报告目录", command=self.open_report, state="disabled")
        self.open_button.grid(row=7, column=0, sticky="w", pady=8)
        ttk.Label(self, text="建议顺序：0 B/s 180 秒 → 32000 B/s 600 秒 → 通过后向上探索。\n缓存不足也是测试结论。通过档位仅适用于本次设备、缓存、分块和上位机组合。", justify="left").grid(row=8, column=0, sticky="w", pady=8)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(6, weight=1)

    def probe(self):
        try:
            user, _, _ = self.identity()
            self.manager.probe_stress(user)
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))

    def start(self):
        try:
            user, training, firmware = self.identity()
            rate, duration = int(self.rate.get()), int(self.duration.get())
            if rate not in RATES or not 1 <= duration <= 3600:
                raise ValueError("选择有效档位，时长需为 1–3600 秒")
            self.manager.start_stress(rate, duration, user, training, firmware, self.host_version)
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))

    def handle(self, event):
        state = event.get("state")
        busy = state in {"preparing", "running", "stopping", "sample"}
        self.busy = busy
        if state == "available":
            self.available = True
        elif state in {"unavailable", "disconnected"}:
            self.available = False
        if state in {"running", "preparing"}:
            self.points = []
            self.report = ""
            self.open_button.configure(state="disabled")
        self.rate_box.configure(state="disabled" if busy else "readonly")
        self.duration_box.configure(state="disabled" if busy else "normal")
        self.query_button.configure(state="disabled" if busy else "normal")
        self.start_button.configure(state="normal" if self.available and not busy else "disabled")
        self.stop_button.configure(state="normal" if state in {"running", "sample"} else "disabled")
        if state == "sample":
            s = event["sample"]
            elapsed = max(s["elapsed_ms"], 1)
            self.metrics.set(
                f"目标 {s['rate_Bps']:,} B/s  |  平均产生 {s['generated_bytes']*1000/elapsed:,.0f} B/s  |  接收 {event.get('receive_Bps', 0):,} B/s\n"
                f"缓存 {s['ring_bytes']:,} / 4096 B（峰值 {s['ring_peak_bytes']:,}）  |  Flash 待传 {s['pending_bytes']:,} B  |  ACK 最大 {s['ack_max_ms']} ms\n"
                f"应产生 {s['expected_bytes']:,} B  |  已产生 {s['generated_bytes']:,} B  |  已确认 {s['acked_bytes']:,} B  |  已运行 {s['elapsed_ms']/1000:.1f} 秒\n"
                f"首个失败：{FAILURES.get(s['failure_code'], s['failure_code'])}")
            self.points.append((s["elapsed_ms"], s["pending_bytes"]))
            self.points = self.points[-300:]
            self.draw()
            return
        if state == "ended":
            self.report = event.get("report") or ""
            self.message.set(f"本场已收尾：{event.get('verdict', '需复核')}。完整性结论来自落盘文件复核。")
            self.open_button.configure(state="normal" if self.report else "disabled")
        else:
            self.message.set(event.get("message") or {"running": "正在压测；到达设定时长后自动停止并收尾。"}.get(state, state))

    def draw(self):
        self.chart.delete("all")
        if len(self.points) < 2:
            return
        width, height = max(self.chart.winfo_width(), 200), max(self.chart.winfo_height(), 100)
        first, last = self.points[0][0], self.points[-1][0]
        peak = max(1, max(v for _, v in self.points))
        coords = []
        for ms, value in self.points:
            coords.extend((12 + (ms-first)/max(1, last-first)*(width-24), height-16-value/peak*(height-40)))
        self.chart.create_line(*coords, fill="#2463c7", width=2)
        self.chart.create_text(12, 10, anchor="nw", text=f"峰值 {peak:,} B", fill="#30415a")

    def report_ready(self, event):
        if self.busy:
            return  # A late storage callback must not replace the next session UI.
        self.report = event["stress_report"]
        self.open_button.configure(state="normal")
        self.message.set(f"本场报告已保存：{event.get('stress_verdict', '需复核')}")

    def open_report(self):
        if self.report:
            os.startfile(os.path.dirname(self.report))
