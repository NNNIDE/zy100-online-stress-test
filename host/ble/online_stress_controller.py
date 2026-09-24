from __future__ import annotations

import asyncio
import time

from .online_stress import CAP_STRESS, CMD_STRESS, QUERY_DETAIL, RATES
from .zy100_protocol import CMD_START_CAPTURE, CMD_PAUSE_CAPTURE, DEVICE_STATE_WAIT_START, build_zy100_command


class OnlineStressController:
    """Connection-bound configuration; ordinary manager owns START/STOP/save/ACK."""

    def __init__(self, manager):
        self.manager = manager
        self.pending = None
        self.timer = None
        self.config = None
        self.client = None
        self.available = False
        self.preparing = False
        self.running = False
        self.session_id = None
        self.start_accepted = False

    def emit(self, state, **values):
        self.manager._emit({"type": "stress_status", "state": state, **values})

    def reset(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.pending is not None and not self.pending[3].done():
            self.pending[3].cancel()
        if hasattr(self.manager, "_online_receiver"):
            self.manager._online_receiver.stress_enabled = False
        self.pending = self.timer = self.config = self.client = None
        self.available = self.preparing = self.running = False
        self.session_id = None
        self.start_accepted = False
        self.emit("disconnected", message="连接已结束，重新查询能力后才能开场")

    def check_idle(self):
        m = self.manager
        if (m._client is None or not m._client.is_connected
                or not m._connection_business_ready or not m._central_business_ready()
                or m._last_device_state != DEVICE_STATE_WAIT_START
                or m._offline_priority_blocked or m._pending_command_active
                or m._online_active or not m._is_calibration_business_ready()):
            raise RuntimeError("请等待 Business Ready、WAIT_START 和离线数据处理完成")

    async def request(self, operation, argument, user_id):
        m = self.manager
        client = m._client
        seq = m._reserve_control_seq()
        future = asyncio.get_running_loop().create_future()
        token = (client, seq, operation, future, user_id)
        self.pending = token
        try:
            await m._write_central_control_frame(build_zy100_command(
                CMD_STRESS, seq, user_id=user_id, training_id=operation,
                device_time_ms=argument), label="ONLINE_STRESS")
            ack = await asyncio.wait_for(future, 5.0)
            if m._client is not client or not client.is_connected:
                raise RuntimeError("配置期间连接已变化")
            if ack.status != 0:
                if ack.status == 3:
                    self.available = False
                    raise RuntimeError("当前固件不支持链路压测，请烧录独立压测固件")
                raise RuntimeError(f"设备拒绝压测配置：status={ack.status} detail={ack.detail}")
            return ack
        finally:
            if self.pending is token:
                self.pending = None

    def handle_ack(self, ack):
        if ack.cmd_echo != CMD_STRESS:
            return False
        token = self.pending
        if token is not None:
            client, seq, operation, future, user_id = token
            if (client is self.manager._client and client.is_connected and ack.seq_echo == seq
                    and ack.training_id == operation and ack.user_id == user_id and not future.done()):
                future.set_result(ack)
        return True

    async def probe(self, user_id=1):
        if self.preparing or self.running:
            return
        try:
            self.check_idle()
            self.preparing = True
            ack = await self.request(0, 0, user_id)
            if ack.detail != QUERY_DETAIL:
                raise RuntimeError(f"压测协议或资源条件不匹配：0x{ack.detail:08X}")
            self.available = True
            self.client = self.manager._client
            self.emit("available", message="协议 v1 / 20 ms / 固定 4096 B 缓存", rates=list(RATES))
        except Exception as exc:
            self.available = False
            self.emit("unavailable", message=str(exc))
        finally:
            self.preparing = False

    async def start(self, rate, duration, user_id, training_id, firmware, host_version):
        if self.preparing or self.running:
            return
        try:
            self.check_idle()
            if rate not in RATES or not 1 <= duration <= 3600:
                raise ValueError("选择有效档位，时长需为 1–3600 秒")
            self.preparing = True
            self.emit("preparing", message="查询能力、确认配置并协商压测协议")
            self.start_accepted = False
            ack = await self.request(0, 0, user_id)
            if ack.detail != QUERY_DETAIL:
                raise RuntimeError("压测能力不匹配")
            client = self.manager._client
            self.client = client
            self.config = {"rate_Bps": rate, "duration_s": duration, "period_ms": 20,
                           "ring_bytes": 4096, "firmware": firmware, "host_version": host_version,
                           "device_address": self.manager._connected_address,
                           "user_id": user_id, "training_id": training_id}
            self.manager._online_ready_sent = False
            self.manager._online_ready_ack_ok = False
            if not await self.manager._send_online_stream_ready_if_possible("stress_negotiate", wait_ack=True):
                raise RuntimeError("压测能力协商未完成")
            ack = await self.request(1, (1 << 32) | rate, user_id)
            if ack.detail != rate:
                raise RuntimeError("设备配置确认档位不一致")
            if self.manager._client is not client:
                raise RuntimeError("开始前连接已变化")
            self.manager._online_receiver.stress_enabled = True
            self.preparing = False
            self.running = True
            await self.manager._send_zy100_command(cmd=CMD_START_CAPTURE, user_id=user_id,
                training_id=training_id, device_time_ms=int(time.time()*1000))
            # START may finish asynchronously; only a matching ONLINE_START starts the duration timer.
            if (not self.start_accepted and self.manager._online_start_seq is None
                    and not self.manager._online_active):
                raise RuntimeError("普通 START 准入未通过，未开始压测")
        except Exception as exc:
            self.running = False
            self.config = None
            self.manager._online_receiver.stress_enabled = False
            self.emit("failed", message=str(exc))
        finally:
            self.preparing = False

    def on_start_ack(self):
        if self.config is not None and self.client is self.manager._client:
            self.start_accepted = True

    def on_start(self, session_id):
        if self.config is None or self.client is not self.manager._client:
            return
        self.session_id = session_id
        self.running = True
        self.timer = asyncio.create_task(self._stop_after(session_id, self.client, self.config["duration_s"]))
        self.emit("running", configuration=dict(self.config), session_id=session_id)

    async def _stop_after(self, session_id, client, seconds):
        await asyncio.sleep(seconds)
        if self.running and self.session_id == session_id and self.manager._client is client:
            await self.stop()

    async def stop(self):
        if not self.running or self.config is None or self.client is not self.manager._client:
            return
        self.emit("stopping", message="停止产生，等待尾部写入、保存和 ACK")
        await self.manager._send_zy100_command(cmd=CMD_PAUSE_CAPTURE,
            user_id=self.config["user_id"], training_id=self.config["training_id"], device_time_ms=None)

    def terminal(self, state, **values):
        if self.timer is not None and self.timer is not asyncio.current_task():
            self.timer.cancel()
        self.timer = None
        self.running = False
        self.config = None
        self.session_id = None
        self.manager._online_receiver.stress_enabled = False
        self.emit(state, **values)
