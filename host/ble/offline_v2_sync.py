from __future__ import annotations

import asyncio
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any

from .offline_v2_protocol import (
    CMD_OFFLINE_CHUNK_ACK,
    CMD_OFFLINE_FINAL_CONFIRM,
    CMD_OFFLINE_RECLAIM_STATUS,
    CMD_OFFLINE_SESSION_BEGIN,
    CMD_OFFLINE_SESSION_LIST,
    CMD_OFFLINE_SESSION_RESUME,
    OFFLINE_FLAG_LAST,
    OFFLINE_FRAME_RECLAIM_STATUS,
    OFFLINE_FRAME_SESSION_BEGIN,
    OFFLINE_FRAME_SESSION_CHUNK,
    OFFLINE_FRAME_SESSION_END,
    OFFLINE_FRAME_SESSION_LIST,
    OFFLINE_FRAME_SYNC_ABORT,
    RECLAIM_ERROR,
    RECLAIM_TOMBSTONE,
    OfflineOuterFrame,
    OfflineSessionListEntry,
    OfflineSessionManifest,
    build_chunk_ack_command,
    build_final_confirm_command,
    build_reclaim_status_command,
    build_session_begin_command,
    build_session_list_command,
    build_session_resume_command,
    parse_reclaim_status,
    parse_session_chunk,
    parse_session_end,
    parse_session_list,
    parse_session_manifest,
    parse_sync_abort,
)
from .offline_v2_store import OfflineV2SessionStore
from .zy100_protocol import ZY100Ack


CommandBuilder = Callable[[int], bytes]
SeqReservedCallback = Callable[[int], None]
CommandSender = Callable[
    [CommandBuilder, str, SeqReservedCallback | None], Awaitable[int | None]
]
EventCallback = Callable[[dict[str, Any]], None]
LogCallback = Callable[[str], None]
CiRestoreWaiter = Callable[[], Awaitable[bool]]

ACK_STATUS_OK = 0
ACK_STATUS_BUSY = 4
ACK_STATUS_INVALID_STATE = 7
ACK_DETAIL_OFFSET = 0x3003
EXEC_MODE_ACCEPTED_ASYNC = 3
EXEC_MODE_DRY_RUN = 1
CHECKPOINT_BYTES = 4 * 1024
MAX_LIST_RETRIES = 3
MAX_BEGIN_RETRIES = 3
CHUNK_ACK_TIMEOUT_SECONDS = 5.0
LIST_BUSY_RETRY_SECONDS = 0.25
LIST_BUSY_DEADLINE_SECONDS = 5.0
RECLAIM_SETTLE_SECONDS = 0.2
RECLAIM_QUERY_INTERVAL_SECONDS = 0.5
RECLAIM_RESPONSE_TIMEOUT_SECONDS = 2.0
MAX_RECLAIM_QUERY_TIMEOUTS = 3
OFFLINE_SYNC_ABORT_CONTRACT = 3
OFFLINE_SYNC_ABORT_ACK_TIMEOUT = 6
OFFLINE_SYNC_ABORT_LOCAL_CAPTURE = 7


@dataclass(slots=True)
class _ChunkAckFlight:
    token: int
    offset: int
    prefix_crc32: int
    seq: int | None = None
    write_returned: bool = False
    consumed: bool = False


@dataclass(slots=True)
class _ResumeFlight:
    token: int
    offset: int
    prefix_crc32: int
    seq: int | None = None
    write_returned: bool = False
    accepted: bool = False


class OfflineV2SyncState(str, Enum):
    IDLE = "idle"
    LISTING = "listing"
    BEGIN = "begin"
    RESUMING = "resuming"
    RECEIVING = "receiving"
    LOCAL_FINALIZE = "local_finalize"
    FINAL_CONFIRM = "final_confirm"
    RECLAIMING = "reclaiming"
    COMPLETE = "complete"
    WAIT_CAPTURE = "wait_capture"
    WAIT_RECONNECT = "wait_reconnect"
    ERROR = "error"


class OfflineV2SyncController:
    def __init__(
        self,
        *,
        store: OfflineV2SessionStore,
        command_sender: CommandSender,
        event_callback: EventCallback,
        log_callback: LogCallback,
        wait_for_balanced: CiRestoreWaiter | None = None,
    ) -> None:
        self.store = store
        self._transport_send = command_sender
        self._emit = event_callback
        self._log = log_callback
        self._wait_for_balanced = wait_for_balanced
        self.state = OfflineV2SyncState.IDLE
        self.connected = False
        self.device_name = ""
        self.device_address = ""
        self.list_generation = 0
        self.entries: list[OfflineSessionListEntry] = []
        self.target: OfflineSessionListEntry | None = None
        self.manifest: OfflineSessionManifest | None = None
        self.session_key: str | None = None
        self.last_acked_offset = 0
        self._pending_ack: tuple[int, int] | None = None
        self._ack_task: asyncio.Task[None] | None = None
        self._chunk_ack_timeout_task: asyncio.Task[None] | None = None
        self._chunk_ack_request_token = 0
        self._chunk_ack_flight: _ChunkAckFlight | None = None
        self._last_rejected_ack_offset: int | None = None
        self._resume_timeout_task: asyncio.Task[None] | None = None
        self._resume_request_token = 0
        self._resume_flight: _ResumeFlight | None = None
        self._abort_resume_task: asyncio.Task[None] | None = None
        self._abort_resume_identity: tuple[int, int, int] | None = None
        self._reclaim_task: asyncio.Task[None] | None = None
        self._reclaim_response_event = asyncio.Event()
        self._list_timeout_task: asyncio.Task[None] | None = None
        self._list_request_token = 0
        self._list_waiting_token: int | None = None
        self._list_retries = 0
        self._list_busy_started_at: float | None = None
        self._begin_retries = 0
        self._last_list_request = (0, 0)
        self._waiting_for_resumed_begin = False
        self._local_completed = False
        self._resume_offset = 0
        self._resume_zero_retry_used = False
        self._last_ignored_chunk_state: OfflineV2SyncState | None = None
        self._final_confirm_transfer_id: int | None = None
        self._clear_paused = False
        self._clear_verify_only = False
        self._clear_verify_future: asyncio.Future[bool] | None = None
        self._priority_completion_future: asyncio.Future[bool] | None = None
        self.local_priority_supported = False
        self.epoch = 0
        self.notification_floor_ns = 0
        self._directory_restarts = 0
        self._reclaim_busy_retries = 0
        self._send_tasks: set[asyncio.Task] = set()
        self._retry_tasks: set[asyncio.Task] = set()
        self._control_sequences: dict[int, int] = {}
        self._clear_discard_task: asyncio.Task | None = None
        self._finalize_task: asyncio.Task | None = None
        self._capture_resume_task: asyncio.Task | None = None
        self._balanced_task: asyncio.Task | None = None
        self._retired_transfers: set[tuple[int, int]] = set()

    @property
    def priority_in_progress(self) -> bool:
        if self.state == OfflineV2SyncState.COMPLETE:
            future = self._priority_completion_future
            return future is not None and not future.done() and not self._clear_paused
        return self.state not in {
            OfflineV2SyncState.IDLE, OfflineV2SyncState.COMPLETE,
            OfflineV2SyncState.ERROR, OfflineV2SyncState.WAIT_RECONNECT,
        } and not self._clear_paused

    async def _send_command(self, builder, label, on_seq_reserved=None):
        epoch = self.epoch
        if not self.connected or self.state == OfflineV2SyncState.WAIT_CAPTURE:
            raise asyncio.CancelledError("offline transaction suspended")

        def reserve(seq):
            self._control_sequences[builder(seq)[2]] = seq
            if on_seq_reserved is not None:
                on_seq_reserved(seq)

        if not self.local_priority_supported:
            return await self._transport_send(builder, label, reserve)
        task = asyncio.create_task(self._transport_send(builder, label, reserve))
        self._send_tasks.add(task)
        try:
            result = await task
            if epoch != self.epoch:
                raise asyncio.CancelledError("obsolete offline transaction")
            return result
        finally:
            self._send_tasks.discard(task)

    def _schedule_retry(self, coroutine) -> None:
        task = asyncio.create_task(coroutine)
        self._retry_tasks.add(task)
        task.add_done_callback(self._retry_tasks.discard)

    def pause_for_local_capture(self) -> bool:
        if not self.local_priority_supported or not self.priority_in_progress:
            return False
        if self.state == OfflineV2SyncState.WAIT_CAPTURE:
            # A second capture may arrive while resume waits for local finalize.
            self._cancel_task(self._capture_resume_task)
            self._capture_resume_task = None
            return True
        self.epoch += 1
        self.notification_floor_ns = time.perf_counter_ns()
        self._control_sequences.clear()
        if self.manifest is not None:
            self._retired_transfers.add((self.manifest.session_id, self.manifest.transfer_id))
        self._set_state(OfflineV2SyncState.WAIT_CAPTURE, reason="local_capture")
        for task in (*self._send_tasks, *self._retry_tasks,
                     self._ack_task, self._abort_resume_task, self._reclaim_task, self._balanced_task):
            self._cancel_task(task)
        self._ack_task = self._abort_resume_task = self._reclaim_task = None
        self._pending_ack = None
        self._invalidate_chunk_ack_request()
        self._invalidate_resume_request()
        self._invalidate_list_request()
        try:
            if self.target is not None and self.target.state != 2:
                self.store.mark_capture_preempted(self.target, device_address=self.device_address)
        except (OSError, ValueError) as exc:
            self._fail("preemption_checkpoint_failed", str(exc))
            return False
        self._emit_status("capture_preempted", session_id=self.target.session_id if self.target else 0)
        # Keep the original bootstrap/Observer completion future pending.
        return True

    def resume_after_local_capture(self) -> None:
        if self.state != OfflineV2SyncState.WAIT_CAPTURE:
            return
        if self._capture_resume_task is None or self._capture_resume_task.done():
            self._capture_resume_task = asyncio.create_task(self._resume_after_capture(self.epoch))

    async def _resume_after_capture(self, epoch: int) -> None:
        try:
            if self._finalize_task is not None:
                await asyncio.shield(self._finalize_task)
            if not self.connected or self.epoch != epoch:
                return
            self.store.close_active()
            self._set_state(OfflineV2SyncState.IDLE, reason="capture_finished")
            self._emit_status("capture_finished")
            self._directory_restarts = 0
            await self.start_sync(reason="capture_finished_relist")
        except asyncio.CancelledError:
            return
        except (OSError, RuntimeError, ValueError) as exc:
            self._fail("preemption_resume_failed", str(exc))

    def set_connection(self, *, connected: bool, device_name: str = "", device_address: str = "") -> None:
        self.connected = connected
        self.device_name = device_name
        self.device_address = device_address

    async def start_sync(self, *, reason: str = "connection_ready") -> None:
        if not self.connected or self._clear_paused or self.state == OfflineV2SyncState.WAIT_CAPTURE:
            return
        start_epoch = self.epoch
        for task in (self._finalize_task, self._clear_discard_task):
            if task is not None and not task.done():
                await asyncio.shield(task)
            if (self.epoch != start_epoch or not self.connected or self._clear_paused
                    or self.state == OfflineV2SyncState.WAIT_CAPTURE):
                return
        self.epoch += 1
        epoch = self.epoch
        self.notification_floor_ns = time.perf_counter_ns()
        self._control_sequences.clear()
        if (
            self._priority_completion_future is None
            or self._priority_completion_future.done()
        ):
            self._priority_completion_future = (
                asyncio.get_running_loop().create_future()
            )
        self.list_generation = 0
        self.entries.clear()
        self.target = None
        self.session_key = None
        self.manifest = None
        self._waiting_for_resumed_begin = False
        self._local_completed = False
        self._resume_offset = 0
        self._last_ignored_chunk_state = None
        self._resume_zero_retry_used = False
        self._cancel_chunk_ack_timeout()
        self._invalidate_resume_request()
        self._cancel_task(self._abort_resume_task)
        self._abort_resume_task = None
        self._abort_resume_identity = None
        self._cancel_task(self._reclaim_task)
        self._reclaim_task = None
        self._last_rejected_ack_offset = None
        self._invalidate_list_request()
        self._list_retries = 0
        self._list_busy_started_at = None
        self._begin_retries = 0
        self._final_confirm_transfer_id = None
        self._reclaim_busy_retries = 0
        self._set_state(OfflineV2SyncState.LISTING, reason=reason)
        try:
            await self._send_list(generation=0, cursor=0)
        except asyncio.CancelledError:
            if self.epoch == epoch:
                raise

    async def wait_for_priority_completion(self) -> bool:
        future = self._priority_completion_future
        if future is None or not self.connected or self._clear_paused:
            return False
        completed = await asyncio.shield(future)
        return bool(completed and self.connected and not self._clear_paused
                    and future is self._priority_completion_future)

    async def pause_for_clear(self) -> None:
        self._complete_priority(False)
        self._priority_completion_future = None
        self.epoch += 1
        self._control_sequences.clear()
        for task in (*self._send_tasks, *self._retry_tasks):
            self._cancel_task(task)
        self._clear_paused = True
        self._clear_verify_only = False
        self._cancel_task(self._ack_task)
        self._ack_task = None
        self._cancel_chunk_ack_timeout()
        self._invalidate_resume_request()
        self._cancel_task(self._abort_resume_task)
        self._abort_resume_task = None
        self._abort_resume_identity = None
        self._cancel_task(self._reclaim_task)
        self._reclaim_task = None
        self._invalidate_list_request()
        self._pending_ack = None
        self._set_state(OfflineV2SyncState.IDLE, reason="clear_requested")

    async def accept_clear(self) -> str | None:
        epoch = self.epoch
        if self._finalize_task is not None and not self._finalize_task.done():
            await asyncio.shield(self._finalize_task)
        if epoch != self.epoch:
            return None
        self._clear_discard_task = asyncio.create_task(
            asyncio.to_thread(self.store.discard_active_for_device_clear))
        path = await asyncio.shield(self._clear_discard_task)
        if epoch != self.epoch:
            return str(path) if path is not None else None
        self.target = None
        self.manifest = None
        self.session_key = None
        self.entries.clear()
        return str(path) if path is not None else None

    async def resume_after_clear_rejected(self) -> None:
        self._clear_paused = False
        self._clear_verify_only = False
        await self.start_sync(reason="clear_rejected_resume")

    async def verify_empty_after_clear(self) -> bool:
        if not self.connected:
            return False
        loop = asyncio.get_running_loop()
        self._clear_paused = True
        self._clear_verify_only = True
        future = loop.create_future()
        self._clear_verify_future = future
        self.list_generation = 0
        self.entries.clear()
        self.target = None
        self._list_retries = 0
        self._set_state(OfflineV2SyncState.LISTING, reason="clear_verify_list")
        await self._send_list(generation=0, cursor=0)
        return await future

    def finish_clear_success(self) -> None:
        self._clear_paused = False
        self._clear_verify_only = False
        self._clear_verify_future = None
        self._invalidate_list_request()
        self._set_state(OfflineV2SyncState.COMPLETE, reason="clear_verified_empty")

    async def on_frame(self, frame: OfflineOuterFrame) -> None:
        epoch = self.epoch
        if self.state == OfflineV2SyncState.WAIT_CAPTURE or not self.connected:
            return
        if self.local_priority_supported:
            if frame.frame_type in {OFFLINE_FRAME_SESSION_BEGIN, OFFLINE_FRAME_SESSION_CHUNK,
                                    OFFLINE_FRAME_SESSION_END, OFFLINE_FRAME_SYNC_ABORT}:
                identity = (int.from_bytes(frame.payload[:4], "little"),
                            int.from_bytes(frame.payload[4:8], "little"))
                if identity in self._retired_transfers:
                    return
        if self._clear_paused and not self._clear_verify_only:
            self._log(f"[OFFLINE_V2][CLEAR] late_frame_ignored type=0x{frame.frame_type:02X}")
            return
        try:
            if frame.frame_type == OFFLINE_FRAME_SESSION_LIST:
                await self._handle_list(frame)
            elif frame.frame_type == OFFLINE_FRAME_SESSION_BEGIN:
                await self._handle_begin(frame)
            elif frame.frame_type == OFFLINE_FRAME_SESSION_CHUNK:
                await self._handle_chunk(frame)
            elif frame.frame_type == OFFLINE_FRAME_SESSION_END:
                await self._handle_end(frame)
            elif frame.frame_type == OFFLINE_FRAME_RECLAIM_STATUS:
                await self._handle_reclaim(frame)
            elif frame.frame_type == OFFLINE_FRAME_SYNC_ABORT:
                await self._handle_abort(frame)
        except asyncio.CancelledError:
            if epoch == self.epoch:
                raise
        except (OSError, RuntimeError, ValueError) as exc:
            if epoch == self.epoch:
                self._fail("protocol_or_storage", str(exc))

    async def on_ack(self, ack: ZY100Ack, *, epoch: int | None = None) -> None:
        if epoch is not None and epoch != self.epoch:
            return
        if self.state == OfflineV2SyncState.WAIT_CAPTURE or not self.connected:
            return
        if self.local_priority_supported and self._control_sequences.get(ack.cmd_echo) != ack.seq_echo:
            return
        if self._clear_paused and not self._clear_verify_only:
            return
        if ack.cmd_echo == CMD_OFFLINE_SESSION_LIST:
            if self.local_priority_supported and ack.status == ACK_STATUS_INVALID_STATE and ack.detail == 0x3002:
                await self._restart_directory()
                return
            if ack.status == ACK_STATUS_BUSY:
                self._schedule_list_retry("device_busy")
            elif ack.status not in {ACK_STATUS_OK}:
                self._complete_clear_verify(False)
                if not self._clear_verify_only:
                    self._fail("list_rejected", f"status={ack.status} detail=0x{ack.detail:08X}")
            return
        if ack.cmd_echo == CMD_OFFLINE_SESSION_BEGIN:
            if ack.status == ACK_STATUS_BUSY and self.target is not None:
                if self._begin_retries < MAX_BEGIN_RETRIES:
                    self._begin_retries += 1
                    self._schedule_retry(self._retry_begin_after_busy(self._begin_retries))
                else:
                    self._fail("begin_busy_exhausted", "BEGIN remained BUSY after LIST serialization")
            elif ack.status not in {ACK_STATUS_OK}:
                self._fail("begin_rejected", f"status={ack.status} detail=0x{ack.detail:08X}")
            return
        if ack.cmd_echo == CMD_OFFLINE_SESSION_RESUME:
            flight = self._resume_flight
            if flight is None or flight.seq != ack.seq_echo:
                self._log(
                    "[OFFLINE_V2][RESUME_ACK_IGNORED] "
                    f"seq={ack.seq_echo} status={ack.status} reason=no_matching_request"
                )
                return
            if ack.status == ACK_STATUS_OK and (
                self.manifest is None
                or int(ack.training_id) != flight.offset
                or int(ack.detail) != self.manifest.transfer_id
            ):
                self._invalidate_resume_request()
                self._enter_wait_reconnect(
                    "resume_ack_identity_mismatch",
                    "SESSION_RESUME ACK does not match offset/transfer identity",
                )
                return
            flight.accepted = ack.status == ACK_STATUS_OK
            early = not flight.write_returned
            self._invalidate_resume_request()
            if ack.status != ACK_STATUS_OK:
                if (self.local_priority_supported and flight.offset > 0 and
                        ack.status == ACK_STATUS_INVALID_STATE and ack.detail == ACK_DETAIL_OFFSET and
                        not self._resume_zero_retry_used and self.target is not None and
                        self.target.state != 2 and self.manifest is not None):
                    # The device may have recorded a preemption while this Host
                    # was disconnected or before it persisted the local marker.
                    # Only an explicit offset rejection permits this one fallback.
                    self._resume_zero_retry_used = True
                    try:
                        self.store.mark_capture_preempted(self.target, device_address=self.device_address)
                        self.store.prepare(self.manifest, list_entry=self.target,
                                           device_name=self.device_name, device_address=self.device_address)
                    except (OSError, RuntimeError, ValueError) as exc:
                        self._fail("resume_zero_prepare_failed", str(exc))
                        return
                    self._local_completed = False
                    self._resume_offset = self.last_acked_offset = 0
                    self._waiting_for_resumed_begin = True
                    self._emit_status("resume_reset_required", session_id=self.manifest.session_id)
                    await self._send_resume(self.manifest, offset=0, prefix_crc32=0)
                    return
                self._enter_wait_reconnect(
                    "resume_rejected",
                    f"status={ack.status} detail=0x{ack.detail:08X}",
                )
            else:
                self._log(
                    "[OFFLINE_V2][RESUME] phase=resume_accepted "
                    f"seq={ack.seq_echo} offset={flight.offset} early={int(early)}"
                )
                self._emit_status("resume_accepted", offset=flight.offset)
            return
        if ack.cmd_echo == CMD_OFFLINE_CHUNK_ACK:
            flight = self._chunk_ack_flight
            if flight is None or flight.seq != ack.seq_echo:
                self._log(
                    "[OFFLINE_V2][ACK_IGNORED] "
                    f"seq={ack.seq_echo} status={ack.status} reason=no_matching_request"
                )
                return
            if ack.status == ACK_STATUS_OK and (
                self.manifest is None
                or int(ack.training_id) != flight.offset
                or int(ack.detail) != self.manifest.transfer_id
            ):
                self._invalidate_chunk_ack_request()
                self._fail(
                    "chunk_ack_identity_mismatch",
                    "CHUNK_ACK response does not match offset/transfer identity",
                )
                return
            if ack.status == ACK_STATUS_OK:
                self.last_acked_offset = max(self.last_acked_offset, int(ack.training_id))
                flight.consumed = True
                early = not flight.write_returned
                self._invalidate_chunk_ack_request()
                if (
                    self._last_rejected_ack_offset is not None
                    and int(ack.training_id) > self._last_rejected_ack_offset
                ):
                    self._last_rejected_ack_offset = None
                self._log(
                    "[OFFLINE_V2][ACK] "
                    f"phase={'ack_consumed_before_write_return' if early else 'acknowledged'} "
                    f"seq={ack.seq_echo} offset={flight.offset}"
                )
                self._ensure_ack_task()
                self._emit_status("acknowledged", offset=self.last_acked_offset)
            elif ack.status == ACK_STATUS_INVALID_STATE and ack.detail == ACK_DETAIL_OFFSET:
                rejected_offset = flight.offset
                flight.consumed = True
                self._invalidate_chunk_ack_request()
                self._last_rejected_ack_offset = rejected_offset
                self._log(
                    "[OFFLINE_V2][ACK_REJECTED] "
                    f"seq={ack.seq_echo} offset={rejected_offset} action=wait_progress"
                )
                # Old firmware may continue beyond an obsolete cumulative ACK.
                # Retry only after a strictly newer durable checkpoint exists.
                self._queue_latest_durable_ack()
            else:
                self._invalidate_chunk_ack_request()
                self._fail("chunk_ack_rejected", f"status={ack.status} detail=0x{ack.detail:08X}")
            return
        if ack.cmd_echo == CMD_OFFLINE_FINAL_CONFIRM:
            if self.local_priority_supported and (
                    self.state != OfflineV2SyncState.FINAL_CONFIRM or self.target is None or
                    (ack.status == ACK_STATUS_OK and ack.training_id != self.target.generation)):
                return
            if ack.status != ACK_STATUS_OK:
                self._fail("final_confirm_rejected", f"status={ack.status} detail=0x{ack.detail:08X}")
                return
            if ack.exec_mode in {EXEC_MODE_ACCEPTED_ASYNC, EXEC_MODE_DRY_RUN}:
                self._set_state(OfflineV2SyncState.RECLAIMING, reason="final_confirm_accepted")
                self._mark_reclaim("reclaiming")
                self._schedule_reclaim_query()
            return
        if ack.cmd_echo == CMD_OFFLINE_RECLAIM_STATUS and ack.status not in {ACK_STATUS_OK}:
            if (self.local_priority_supported and ack.status == ACK_STATUS_BUSY and
                    self.state == OfflineV2SyncState.RECLAIMING and
                    self._reclaim_busy_retries < MAX_LIST_RETRIES):
                self._reclaim_busy_retries += 1
                self._schedule_reclaim_query()
                return
            self._fail("reclaim_query_rejected", f"status={ack.status} detail=0x{ack.detail:08X}")

    async def on_disconnect(self) -> None:
        if not self.connected and self.state in {
            OfflineV2SyncState.IDLE,
            OfflineV2SyncState.WAIT_RECONNECT,
        }:
            return
        self.connected = False
        self.epoch += 1
        disconnect_epoch = self.epoch
        self._complete_priority(False)
        self._priority_completion_future = None
        self._complete_clear_verify(False)
        self._clear_verify_future = None
        self._clear_paused = False
        self._clear_verify_only = False
        self._control_sequences.clear()
        self._retired_transfers.clear()
        self._cancel_task(self._capture_resume_task)
        self._cancel_task(self._balanced_task)
        for task in (*self._send_tasks, *self._retry_tasks):
            self._cancel_task(task)
        for task in (self._finalize_task, self._clear_discard_task):
            if task is not None and not task.done():
                try:
                    await asyncio.shield(task)
                except (OSError, RuntimeError, ValueError) as exc:
                    self._log(f"[OFFLINE_V2][DISCONNECT] local_store_failed={exc}")
        if disconnect_epoch != self.epoch:
            return
        self._cancel_task(self._ack_task)
        self._ack_task = None
        self._cancel_chunk_ack_timeout()
        self._invalidate_resume_request()
        self._cancel_task(self._abort_resume_task)
        self._abort_resume_task = None
        self._abort_resume_identity = None
        self._cancel_task(self._reclaim_task)
        self._reclaim_task = None
        self._invalidate_list_request()
        self._pending_ack = None
        self._last_rejected_ack_offset = None
        self.store.close_active()
        self._complete_clear_verify(False)
        self._complete_priority(False)
        self._set_state(OfflineV2SyncState.WAIT_RECONNECT, reason="ble_disconnect")

    async def _restart_directory(self) -> None:
        if self.state != OfflineV2SyncState.LISTING:
            return
        self._directory_restarts += 1
        if self._directory_restarts > MAX_LIST_RETRIES:
            self._fail("list_directory_unstable", "directory changed repeatedly")
            return
        self._invalidate_list_request()
        self.list_generation = 0
        self.entries.clear()
        await self._send_list(generation=0, cursor=0)

    async def _handle_list(self, frame: OfflineOuterFrame) -> None:
        page = parse_session_list(frame)
        if self.state != OfflineV2SyncState.LISTING:
            self._log(
                f"[OFFLINE_V2][LIST_IGNORED] state={self.state.value} "
                f"generation={page.generation} cursor={page.cursor}"
            )
            return
        if self.local_priority_supported and page.cursor != self._last_list_request[1]:
            return  # A delayed old page cannot replace the current cursor.
        self._list_waiting_token = None
        self._cancel_list_timeout()
        self._list_retries = 0
        self._list_busy_started_at = None
        if self.list_generation == 0:
            if page.cursor != 0:
                raise ValueError("OFFLINE V2 first list page does not start at cursor zero")
            self.list_generation = page.generation
            self.entries.clear()
        elif page.generation != self.list_generation:
            if self.local_priority_supported:
                await self._restart_directory()
                return
            raise ValueError("OFFLINE V2 list generation changed during pagination")
        if page.cursor != len(self.entries):
            raise ValueError(
                f"OFFLINE V2 list cursor discontinuity: {page.cursor} != {len(self.entries)}"
            )
        self.entries.extend(page.entries)
        self._set_state(OfflineV2SyncState.LISTING, reason="list_page")
        self._emit_status(
            "list_page",
            list_generation=page.generation,
            cursor=page.cursor,
            next_cursor=page.next_cursor,
            session_count=page.total,
        )
        if not page.is_last:
            await self._send_list(generation=page.generation, cursor=page.next_cursor)
            return
        if len(self.entries) != page.total:
            raise ValueError("OFFLINE V2 list page total does not match accumulated entries")
        if not self.entries:
            self._set_state(OfflineV2SyncState.COMPLETE, reason="no_pending_sessions")
            self._emit_status("all_complete", session_count=0)
            self._complete_clear_verify(True)
            self._complete_priority(True)
            return
        if self._clear_verify_only:
            self._emit_status("clear_verify_not_empty", session_count=len(self.entries))
            self._complete_clear_verify(False)
            return
        self._directory_restarts = 0
        self.target = self.entries[0]
        if self.target.state == 2:
            self._set_state(OfflineV2SyncState.RECLAIMING, reason="confirmed_session_found")
            await self._send_reclaim_query()
            if self.connected and self.state == OfflineV2SyncState.RECLAIMING:
                self._schedule_reclaim_query()
            return
        self._set_state(OfflineV2SyncState.BEGIN, reason="device_priority_session_selected")
        await self._send_begin(self.target)

    async def _handle_begin(self, frame: OfflineOuterFrame) -> None:
        manifest = parse_session_manifest(frame)
        if self.local_priority_supported and self.state not in {OfflineV2SyncState.BEGIN, OfflineV2SyncState.RESUMING}:
            return
        if self.target is None:
            raise ValueError("OFFLINE V2 BEGIN arrived without a selected session")
        if (
            manifest.session_id != self.target.session_id
            or manifest.generation != self.target.generation
            or manifest.event_count != self.target.event_count
            or manifest.logical_bytes != self.target.logical_bytes
            or manifest.stream_crc32 != self.target.stream_crc32
        ):
            if self.local_priority_supported:
                return
            raise ValueError("OFFLINE V2 BEGIN does not match LIST entry")

        if self._waiting_for_resumed_begin:
            if self.manifest is None or manifest != self.manifest:
                raise ValueError("OFFLINE V2 resumed BEGIN changed manifest")
            self._invalidate_resume_request()
            self._waiting_for_resumed_begin = False
            self._abort_resume_identity = None
            self._set_state(OfflineV2SyncState.RECEIVING, reason="resume_begin_verified")
            self._log(
                "[OFFLINE_V2][RESUME] phase=resume_begin_verified "
                f"session={manifest.session_id} offset={self._resume_offset}"
            )
            self._emit_status(
                "resumed",
                session_id=manifest.session_id,
                offset=self._resume_offset,
                total_bytes=manifest.logical_bytes,
            )
            return

        self.manifest = manifest
        self._log(
            "[OFF_SESSION][HEALTH_A] "
            f"session={manifest.session_id} health={manifest.health_name} "
            f"stop_reason={manifest.stop_reason} clean={int(manifest.clean)}"
        )
        self._log(
            "[OFF_SESSION][HEALTH_B] "
            f"fifo_overflow={manifest.fifo_overflow_count} "
            f"fifo_discard={manifest.fifo_discard_count} "
            f"time_gap={manifest.time_gap_count} "
            f"feature_drop={manifest.feature_drop_count} "
            f"q12_clip={manifest.q12_clip_count} "
            f"flash_error={manifest.flash_error_count}"
        )
        prepared = self.store.prepare(
            manifest,
            list_entry=self.target,
            device_name=self.device_name,
            device_address=self.device_address,
        )
        self.session_key = prepared.session_key
        self.last_acked_offset = prepared.durable_offset
        self._local_completed = prepared.completed
        self._resume_offset = prepared.durable_offset
        self._waiting_for_resumed_begin = True
        resume_reason = (
            "local_checkpoint_found"
            if prepared.durable_offset > 0 or prepared.completed
            else "initial_resume_barrier"
        )
        self._set_state(OfflineV2SyncState.RESUMING, reason=resume_reason)
        self._log(
            "[OFFLINE_V2][RESUME] phase=initial_barrier "
            f"session={manifest.session_id} offset={prepared.durable_offset}"
        )
        await self._send_resume(
            manifest,
            offset=prepared.durable_offset,
            prefix_crc32=prepared.prefix_crc32,
        )

    async def _handle_chunk(self, frame: OfflineOuterFrame) -> None:
        if self.state in {
            OfflineV2SyncState.RESUMING,
            OfflineV2SyncState.WAIT_RECONNECT,
            OfflineV2SyncState.ERROR,
        }:
            if self._last_ignored_chunk_state != self.state:
                self._log(
                    "[OFFLINE_V2][CHUNK_IGNORED] "
                    f"state={self.state.value} reason=resume_barrier"
                )
                self._last_ignored_chunk_state = self.state
            return
        if self.state != OfflineV2SyncState.RECEIVING or self.manifest is None:
            raise ValueError(f"OFFLINE V2 CHUNK in state {self.state.value}")
        self._last_ignored_chunk_state = None
        chunk = parse_session_chunk(frame)
        if (
            chunk.session_id != self.manifest.session_id
            or chunk.transfer_id != self.manifest.transfer_id
            or chunk.total_bytes != self.manifest.logical_bytes
        ):
            raise ValueError("OFFLINE V2 CHUNK identity does not match manifest")
        result = self.store.append(
            offset=chunk.offset,
            data=chunk.data,
            expected_prefix_crc32=chunk.prefix_crc32,
        )
        force_checkpoint = bool(frame.flags & OFFLINE_FLAG_LAST)
        if force_checkpoint or self.store.received_offset - self.store.durable_offset >= CHECKPOINT_BYTES:
            durable_offset, prefix_crc = self.store.checkpoint()
            self._pending_ack = (durable_offset, prefix_crc)
            self._ensure_ack_task()
            self._emit_status(
                "receiving",
                session_id=self.manifest.session_id,
                offset=durable_offset,
                received_offset=result.received_offset,
                total_bytes=self.manifest.logical_bytes,
                duplicate=result.duplicate,
            )

    async def _handle_end(self, frame: OfflineOuterFrame) -> None:
        if self.state in {
            OfflineV2SyncState.RESUMING,
            OfflineV2SyncState.FINAL_CONFIRM,
            OfflineV2SyncState.RECLAIMING,
            OfflineV2SyncState.COMPLETE,
        }:
            self._log(f"[OFFLINE_V2][END_IGNORED] state={self.state.value}")
            return
        if self.state != OfflineV2SyncState.RECEIVING:
            raise ValueError(f"OFFLINE V2 END in state {self.state.value}")
        if self.manifest is None or self.target is None:
            raise ValueError("OFFLINE V2 END arrived without active manifest")
        end = parse_session_end(frame)
        if self._final_confirm_transfer_id == end.transfer_id:
            self._log(
                f"[OFFLINE_V2][END_IGNORED] state=duplicate transfer={end.transfer_id}"
            )
            return
        self._cancel_chunk_ack_timeout()
        epoch = self.epoch
        self._set_state(OfflineV2SyncState.LOCAL_FINALIZE, reason="end_received")
        self._emit_status("local_finalize", session_id=end.session_id)
        if self._local_completed:
            result_session_key = self.session_key
        else:
            self._finalize_task = asyncio.create_task(asyncio.to_thread(self.store.finalize, end))
            result = await asyncio.shield(self._finalize_task)
            if epoch != self.epoch:
                self._emit({"type": "offline_v2_sessions_changed"})
                return
            result_session_key = result.session_key
            self.session_key = result.session_key
            self._emit({"type": "offline_v2_sessions_changed"})
        self._emit_status(
            "saved_local",
            session_id=end.session_id,
            session_key=result_session_key,
            total_bytes=end.logical_bytes,
            event_count=end.event_count,
            health=self.manifest.health_name,
            stop_reason=self.manifest.stop_reason,
            clean=self.manifest.clean,
        )
        self._log(
            "[OFF_SESSION][VERIFY] "
            f"session={end.session_id} persisted=1 length_crc=ok "
            f"stream_crc=0x{end.stream_crc32:08X} action=final_confirm"
        )
        self._set_state(OfflineV2SyncState.FINAL_CONFIRM, reason="local_commit_complete")
        self._final_confirm_transfer_id = end.transfer_id
        seq = await self._send_final_confirm(end)
        if seq is None:
            self._fail("final_confirm_write_failed", "FINAL_CONFIRM was not written")

    async def _handle_reclaim(self, frame: OfflineOuterFrame) -> None:
        epoch = self.epoch
        reclaim = parse_reclaim_status(frame)
        if self.local_priority_supported and (self.state not in {
                OfflineV2SyncState.FINAL_CONFIRM, OfflineV2SyncState.RECLAIMING} or
                self.target is None or reclaim.session_id != self.target.session_id or
                reclaim.generation != self.target.generation):
            return
        if self.target is not None and (
            reclaim.session_id != self.target.session_id
            or reclaim.generation != self.target.generation
        ):
            raise ValueError("OFFLINE V2 reclaim status identity mismatch")
        self._reclaim_response_event.set()
        self._mark_reclaim(
            reclaim.state_name,
            erased_bytes=reclaim.erased_bytes,
            extent_bytes=reclaim.extent_bytes,
        )
        self._reclaim_busy_retries = 0
        self._emit_status(
            "reclaim_status",
            session_id=reclaim.session_id,
            generation=reclaim.generation,
            reclaim_state=reclaim.state_name,
            erased_bytes=reclaim.erased_bytes,
            extent_bytes=reclaim.extent_bytes,
        )
        if reclaim.state == RECLAIM_TOMBSTONE:
            self._cancel_task(self._reclaim_task)
            self._reclaim_task = None
            self._set_state(OfflineV2SyncState.COMPLETE, reason="device_tombstone")
            await asyncio.sleep(RECLAIM_SETTLE_SECONDS)
            if self.connected and epoch == self.epoch:
                balanced = True
                if self._wait_for_balanced is not None:
                    self._balanced_task = asyncio.create_task(self._wait_for_balanced())
                    balanced = await self._balanced_task
                if not balanced:
                    if epoch != self.epoch:
                        return
                    self._fail(
                        "ci_restore_timeout",
                        "BALANCED CI was not applied after reclaim",
                    )
                    return
                if self.connected and epoch == self.epoch:
                    await self.start_sync(reason="next_session_after_reclaim")
        elif reclaim.state == RECLAIM_ERROR:
            self._fail("reclaim_error", "device reported reclaim error")
        else:
            # PAUSED is a per-session cleanup state, not proof that capture is
            # still running. Runtime capture/ABORT notifications own the quiet
            # wait; a paused task queried after sealing must remain queryable.
            self._set_state(OfflineV2SyncState.RECLAIMING, reason=reclaim.state_name)
            self._schedule_reclaim_query()

    async def _handle_abort(self, frame: OfflineOuterFrame) -> None:
        abort = parse_sync_abort(frame)
        if abort.reason == OFFLINE_SYNC_ABORT_LOCAL_CAPTURE and self.local_priority_supported:
            if self.manifest is not None and (
                    abort.session_id != self.manifest.session_id or
                    abort.generation != self.manifest.generation or
                    abort.transfer_id != self.manifest.transfer_id):
                return
            if self.target is not None and (
                    abort.session_id != self.target.session_id or abort.generation != self.target.generation):
                return
            self.pause_for_local_capture()
            return
        epoch = self.epoch
        self._log(
            "[OFFLINE_V2][ABORT] "
            f"session={abort.session_id} transfer={abort.transfer_id} reason={abort.reason} "
            f"detail=0x{abort.detail:08X} acked={abort.acked_offset}"
        )
        self._emit_status(
            "sync_abort",
            session_id=abort.session_id,
            reason=abort.reason,
            detail=abort.detail,
            offset=self.store.durable_offset,
        )
        self._cancel_task(self._ack_task)
        self._ack_task = None
        self._pending_ack = None
        self._cancel_chunk_ack_timeout()

        if abort.reason != OFFLINE_SYNC_ABORT_ACK_TIMEOUT:
            category = (
                "sync_abort_contract"
                if abort.reason == OFFLINE_SYNC_ABORT_CONTRACT
                else "sync_abort_unsupported"
            )
            self._fail(category, f"reason={abort.reason} detail=0x{abort.detail:08X}")
            return
        manifest = self.manifest
        identity = (abort.session_id, abort.generation, abort.transfer_id)
        if self._abort_resume_identity == identity:
            self._log(
                "[OFFLINE_V2][ABORT] phase=duplicate_ignored "
                f"session={abort.session_id} transfer={abort.transfer_id}"
            )
            return
        if manifest is None or (
            abort.session_id != manifest.session_id
            or abort.generation != manifest.generation
            or abort.transfer_id != manifest.transfer_id
        ):
            self._fail("abort_identity_mismatch", "SYNC_ABORT does not match active manifest")
            return
        try:
            durable_offset, durable_crc = await asyncio.to_thread(
                self.store.validate_active_checkpoint
            )
        except (OSError, RuntimeError, ValueError) as exc:
            if epoch == self.epoch:
                self._fail("abort_checkpoint_invalid", str(exc))
            return
        if epoch != self.epoch:
            return
        if not (
            0 <= abort.acked_offset <= abort.detail <= manifest.logical_bytes
            and 0 <= durable_offset <= abort.detail
        ):
            self._fail(
                "abort_offset_invalid",
                "SYNC_ABORT offsets are inconsistent with the durable checkpoint",
            )
            return
        if not self.connected:
            self._set_state(OfflineV2SyncState.WAIT_RECONNECT, reason="abort_disconnected")
            return

        self._abort_resume_identity = identity
        self._resume_offset = durable_offset
        self._waiting_for_resumed_begin = True
        self._set_state(OfflineV2SyncState.RESUMING, reason="abort_resume_selected")
        self._log(
            "[OFFLINE_V2][ABORT] phase=abort_resume_selected "
            f"session={abort.session_id} transfer={abort.transfer_id} "
            f"device_acked={abort.acked_offset} durable={durable_offset} "
            f"crc={durable_crc:08X}"
        )
        self._cancel_task(self._abort_resume_task)
        self._abort_resume_task = asyncio.create_task(
            self._resume_after_abort(manifest, durable_offset, durable_crc)
        )

    async def _resume_after_abort(
        self,
        manifest: OfflineSessionManifest,
        offset: int,
        prefix_crc32: int,
    ) -> None:
        try:
            if self._wait_for_balanced is not None:
                self._log("[OFFLINE_V2][ABORT] phase=wait_balanced")
                if not await self._wait_for_balanced():
                    self._enter_wait_reconnect(
                        "abort_ci_restore_timeout",
                        "BALANCED CI was not applied before SESSION_RESUME",
                    )
                    return
            if (
                not self.connected
                or self.state != OfflineV2SyncState.RESUMING
                or self.manifest != manifest
            ):
                return
            await self._send_resume(
                manifest,
                offset=offset,
                prefix_crc32=prefix_crc32,
            )
        except asyncio.CancelledError:
            return

    async def _send_list(self, *, generation: int, cursor: int) -> None:
        if self.state != OfflineV2SyncState.LISTING:
            return
        self._cancel_list_timeout()
        self._list_request_token += 1
        request_token = self._list_request_token
        self._list_waiting_token = request_token
        self._last_list_request = (generation, cursor)
        seq = await self._send_command(
            lambda seq: build_session_list_command(
                seq, generation=generation, cursor=cursor, page_size=0
            ),
            "OFFLINE_SESSION_LIST",
            None,
        )
        if self._list_waiting_token != request_token:
            return
        if seq is None:
            self._list_waiting_token = None
            self._schedule_list_retry("write_failed")
        else:
            self._list_timeout_task = asyncio.create_task(
                self._wait_list_timeout(request_token)
            )

    async def _send_begin(self, entry: OfflineSessionListEntry) -> None:
        await self._send_command(
            lambda seq: build_session_begin_command(
                seq, session_id=entry.session_id, generation=entry.generation
            ),
            "OFFLINE_SESSION_BEGIN",
            None,
        )

    async def _retry_begin_after_busy(self, attempt: int) -> None:
        await asyncio.sleep(0.25 * attempt)
        if self.connected and self.target is not None and not self._clear_paused and self.state == OfflineV2SyncState.BEGIN:
            await self._send_begin(self.target)

    async def _wait_list_timeout(self, request_token: int) -> None:
        try:
            await asyncio.sleep(CHUNK_ACK_TIMEOUT_SECONDS)
        except asyncio.CancelledError:
            return
        if self._list_waiting_token != request_token:
            return
        self._list_waiting_token = None
        self._list_timeout_task = None
        self._schedule_list_retry("notify_timeout")

    def _schedule_list_retry(self, reason: str) -> None:
        self._cancel_list_timeout()
        if self.state != OfflineV2SyncState.LISTING:
            return
        if not self.connected:
            self._complete_clear_verify(False)
            return
        loop = asyncio.get_running_loop()
        if reason == "device_busy":
            if self._list_busy_started_at is None:
                self._list_busy_started_at = loop.time()
            if (loop.time() - self._list_busy_started_at) >= LIST_BUSY_DEADLINE_SECONDS:
                self._complete_clear_verify(False)
                if not self._clear_verify_only:
                    self._fail("list_busy_timeout", "LIST remained BUSY for 5 seconds")
                return
        elif self._list_retries >= MAX_LIST_RETRIES:
            self._complete_clear_verify(False)
            if not self._clear_verify_only:
                self._fail("list_retry_exhausted", reason)
            return
        self._list_retries += 1
        generation, cursor = self._last_list_request
        self._schedule_retry(self._retry_list(generation, cursor, self._list_retries, reason))

    async def _retry_list(self, generation: int, cursor: int, attempt: int, reason: str) -> None:
        delay = LIST_BUSY_RETRY_SECONDS if reason == "device_busy" else 0.25 * attempt
        await asyncio.sleep(delay)
        if (
            self.connected
            and self.state == OfflineV2SyncState.LISTING
            and (not self._clear_paused or self._clear_verify_only)
        ):
            self._log(f"[OFFLINE_V2][LIST] retry={attempt} reason={reason}")
            await self._send_list(generation=generation, cursor=cursor)

    def _cancel_list_timeout(self) -> None:
        self._cancel_task(self._list_timeout_task)
        self._list_timeout_task = None

    def _invalidate_list_request(self) -> None:
        self._cancel_list_timeout()
        self._list_request_token += 1
        self._list_waiting_token = None

    def _complete_clear_verify(self, result: bool) -> None:
        future = self._clear_verify_future
        if future is not None and not future.done():
            future.set_result(result)

    async def _send_resume(
        self, manifest: OfflineSessionManifest, *, offset: int, prefix_crc32: int
    ) -> None:
        self._invalidate_resume_request()
        self._resume_request_token += 1
        request_token = self._resume_request_token
        flight = _ResumeFlight(request_token, offset, prefix_crc32)
        self._resume_flight = flight
        try:
            seq = await self._send_command(
                lambda seq: build_session_resume_command(
                    seq,
                    session_id=manifest.session_id,
                    transfer_id=manifest.transfer_id,
                    next_offset=offset,
                    prefix_crc32=prefix_crc32,
                ),
                "OFFLINE_SESSION_RESUME",
                lambda seq: self._bind_resume_seq(request_token, seq),
            )
        except Exception as exc:
            flight.write_returned = True
            if self._resume_flight is flight:
                self._invalidate_resume_request()
            if self.connected:
                self._enter_wait_reconnect("resume_write_failed", str(exc))
            return
        flight.write_returned = True
        if self._resume_flight is not flight:
            if flight.accepted:
                self._log(
                    "[OFFLINE_V2][RESUME] phase=ack_consumed_before_write_return "
                    f"seq={flight.seq} offset={offset}"
                )
            return
        if seq is None or flight.seq != seq:
            self._invalidate_resume_request()
            if self.connected:
                self._enter_wait_reconnect(
                    "resume_write_failed",
                    "OFFLINE SESSION_RESUME was not written",
                )
            return
        self._log(
            "[OFFLINE_V2][RESUME] phase=ack_wait_started "
            f"seq={seq} offset={offset}"
        )
        self._resume_timeout_task = asyncio.create_task(
            self._wait_resume_ack_timeout(request_token, seq, offset)
        )

    def _bind_resume_seq(self, request_token: int, seq: int) -> None:
        flight = self._resume_flight
        if flight is None or flight.token != request_token:
            return
        flight.seq = seq
        self._log(
            "[OFFLINE_V2][RESUME] phase=registered "
            f"token={request_token} seq={seq} offset={flight.offset}"
        )

    async def _wait_resume_ack_timeout(
        self, request_token: int, seq: int, offset: int
    ) -> None:
        try:
            await asyncio.sleep(CHUNK_ACK_TIMEOUT_SECONDS)
        except asyncio.CancelledError:
            return
        flight = self._resume_flight
        if (
            flight is not None
            and flight.token == request_token
            and flight.seq == seq
            and self.connected
        ):
            self._invalidate_resume_request()
            self._enter_wait_reconnect(
                "resume_ack_timeout",
                f"OFFLINE SESSION_RESUME seq={seq} offset={offset} timed out after 5 seconds",
            )

    def _invalidate_resume_request(self) -> None:
        self._cancel_task(self._resume_timeout_task)
        self._resume_timeout_task = None
        self._resume_request_token += 1
        self._resume_flight = None

    async def _send_final_confirm(self, end: Any) -> int | None:
        return await self._send_command(
            lambda seq: build_final_confirm_command(
                seq,
                session_id=end.session_id,
                generation=end.generation,
                transfer_id=end.transfer_id,
                stream_crc32=end.stream_crc32,
            ),
            "OFFLINE_FINAL_CONFIRM",
            None,
        )

    async def _send_reclaim_query(self) -> None:
        if self.target is None or not self.connected or self.state != OfflineV2SyncState.RECLAIMING:
            return
        await self._send_command(
            lambda seq: build_reclaim_status_command(
                seq,
                session_id=self.target.session_id,
                generation=self.target.generation,
            ),
            "OFFLINE_RECLAIM_STATUS",
            None,
        )

    def _ensure_ack_task(self) -> None:
        if self._ack_task is None or self._ack_task.done():
            self._ack_task = asyncio.create_task(self._drain_ack())

    def _queue_latest_durable_ack(self) -> None:
        if self.manifest is None or self.store.durable_offset <= 0:
            return
        if (
            self._last_rejected_ack_offset is not None
            and self.store.durable_offset <= self._last_rejected_ack_offset
        ):
            self._log(
                "[OFFLINE_V2][ACK_COALESCE] "
                f"durable={self.store.durable_offset} "
                f"rejected={self._last_rejected_ack_offset} action=hold"
            )
            return
        self._pending_ack = (self.store.durable_offset, self.store.durable_crc32)
        self._ensure_ack_task()

    async def _drain_ack(self) -> None:
        try:
            if (
                not self.connected
                or self._pending_ack is None
                or self.manifest is None
                or self._chunk_ack_flight is not None
            ):
                return
            await asyncio.sleep(0.03)
            if self._chunk_ack_flight is not None or self._pending_ack is None:
                return
            offset, prefix_crc = self._pending_ack
            self._pending_ack = None
            manifest = self.manifest
            self._chunk_ack_request_token += 1
            request_token = self._chunk_ack_request_token
            flight = _ChunkAckFlight(request_token, offset, prefix_crc)
            self._chunk_ack_flight = flight
            try:
                seq = await self._send_command(
                    lambda seq: build_chunk_ack_command(
                        seq,
                        session_id=manifest.session_id,
                        transfer_id=manifest.transfer_id,
                        next_offset=offset,
                        prefix_crc32=prefix_crc,
                    ),
                    "OFFLINE_CHUNK_ACK",
                    lambda seq: self._bind_chunk_ack_seq(request_token, seq),
                )
            except Exception as exc:
                flight.write_returned = True
                if self._chunk_ack_flight is flight:
                    self._invalidate_chunk_ack_request()
                if self.connected:
                    self._fail("ack_write_failed", str(exc))
                return
            flight.write_returned = True
            if self._chunk_ack_flight is not flight:
                if flight.consumed:
                    self._log(
                        "[OFFLINE_V2][ACK] phase=ack_consumed_before_write_return "
                        f"seq={flight.seq} offset={offset}"
                    )
                    asyncio.get_running_loop().call_soon(self._ensure_ack_task)
                return
            if seq is None or flight.seq != seq:
                self._invalidate_chunk_ack_request()
                if self.connected:
                    self._fail("ack_write_failed", "OFFLINE CHUNK_ACK was not written")
                return
            self._log(
                "[OFFLINE_V2][ACK] phase=ack_wait_started "
                f"token={request_token} seq={seq} offset={offset}"
            )
            self._cancel_task(self._chunk_ack_timeout_task)
            self._chunk_ack_timeout_task = asyncio.create_task(
                self._wait_chunk_ack_timeout(request_token, seq, offset)
            )
        except asyncio.CancelledError:
            return

    def _bind_chunk_ack_seq(self, request_token: int, seq: int) -> None:
        flight = self._chunk_ack_flight
        if flight is None or flight.token != request_token:
            return
        flight.seq = seq
        self._log(
            "[OFFLINE_V2][ACK] phase=registered "
            f"token={request_token} seq={seq} offset={flight.offset}"
        )

    async def _wait_chunk_ack_timeout(
        self, request_token: int, seq: int, offset: int
    ) -> None:
        try:
            await asyncio.sleep(CHUNK_ACK_TIMEOUT_SECONDS)
        except asyncio.CancelledError:
            return
        flight = self._chunk_ack_flight
        if (
            flight is not None
            and flight.token == request_token
            and flight.seq == seq
            and self.connected
        ):
            self._chunk_ack_request_token += 1
            self._chunk_ack_flight = None
            self._chunk_ack_timeout_task = None
            self._fail(
                "ack_timeout",
                f"OFFLINE CHUNK_ACK seq={seq} offset={offset} timed out after 5 seconds",
            )

    def _cancel_chunk_ack_timeout(self) -> None:
        self._invalidate_chunk_ack_request()

    def _invalidate_chunk_ack_request(self) -> None:
        self._cancel_task(self._chunk_ack_timeout_task)
        self._chunk_ack_timeout_task = None
        self._chunk_ack_request_token += 1
        self._chunk_ack_flight = None

    def _schedule_reclaim_query(self) -> None:
        if self._reclaim_task is None or self._reclaim_task.done():
            self._reclaim_task = asyncio.create_task(self._delayed_reclaim_query())

    async def _delayed_reclaim_query(self) -> None:
        epoch = self.epoch
        timeouts = 0
        try:
            while (self.connected and epoch == self.epoch and
                   self.state == OfflineV2SyncState.RECLAIMING):
                await asyncio.sleep(RECLAIM_QUERY_INTERVAL_SECONDS)
                if (not self.connected or epoch != self.epoch or
                        self.state != OfflineV2SyncState.RECLAIMING):
                    return
                self._reclaim_response_event.clear()
                await self._send_reclaim_query()
                if (not self.connected or epoch != self.epoch or
                        self.state != OfflineV2SyncState.RECLAIMING):
                    return
                try:
                    await asyncio.wait_for(
                        self._reclaim_response_event.wait(),
                        RECLAIM_RESPONSE_TIMEOUT_SECONDS,
                    )
                    timeouts = 0
                except asyncio.TimeoutError:
                    if (not self.connected or epoch != self.epoch or
                            self.state != OfflineV2SyncState.RECLAIMING):
                        return
                    timeouts += 1
                    self._log(f"[OFFLINE_V2][RECLAIM] response_timeout count={timeouts}")
                    if timeouts >= MAX_RECLAIM_QUERY_TIMEOUTS:
                        self._enter_wait_reconnect(
                            "reclaim_status_timeout",
                            "device reclaim status did not arrive after repeated queries",
                        )
                        return
        except asyncio.CancelledError:
            return
        except (OSError, RuntimeError, ValueError) as exc:
            if self.connected and epoch == self.epoch:
                self._enter_wait_reconnect("reclaim_query_failed", str(exc))
        finally:
            if self._reclaim_task is asyncio.current_task():
                self._reclaim_task = None

    def _mark_reclaim(self, state: str, *, erased_bytes: int = 0, extent_bytes: int = 0) -> None:
        if self.target is None:
            return
        self.store.update_reclaim_status(
            self.target.session_id,
            self.target.generation,
            state=state,
            erased_bytes=erased_bytes,
            extent_bytes=extent_bytes,
        )
        self._emit({"type": "offline_v2_sessions_changed"})

    def _set_state(self, state: OfflineV2SyncState, *, reason: str) -> None:
        self.state = state
        self._log(f"[OFFLINE_V2][STATE] state={state.value} reason={reason}")

    def _emit_status(self, status: str, **fields: Any) -> None:
        self._emit(
            {
                "type": "offline_v2_status",
                "status": status,
                "state": self.state.value,
                **fields,
            }
        )

    def _fail(self, category: str, reason: str) -> None:
        self._set_state(OfflineV2SyncState.ERROR, reason=category)
        self._log(f"[OFFLINE_V2][ERR] category={category} reason={reason}")
        self._emit_status("error", error_category=category, reason=reason)
        self._complete_priority(False)

    def _enter_wait_reconnect(self, category: str, reason: str) -> None:
        self._set_state(OfflineV2SyncState.WAIT_RECONNECT, reason=category)
        self._log(f"[OFFLINE_V2][ERR] category={category} reason={reason}")
        self._emit_status("error", error_category=category, reason=reason)
        self._complete_priority(False)

    def _complete_priority(self, result: bool) -> None:
        future = self._priority_completion_future
        if future is not None and not future.done():
            future.set_result(result)

    @staticmethod
    def _cancel_task(task: asyncio.Task[Any] | None) -> None:
        if task is not None and not task.done():
            task.cancel()
