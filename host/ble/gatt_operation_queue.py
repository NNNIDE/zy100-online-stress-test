from __future__ import annotations

import asyncio
import heapq
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Awaitable, Callable, Generic, TypeVar


T = TypeVar("T")


class GattPriority(IntEnum):
    CRITICAL = 0
    NORMAL = 10
    BACKGROUND = 20


@dataclass(order=True)
class _QueuedGattOperation(Generic[T]):
    priority: int
    sequence: int
    token: int | None = field(compare=False)
    name: str = field(compare=False)
    operation: Callable[[], Awaitable[T]] = field(compare=False)
    future: asyncio.Future[T] = field(compare=False)


class GattOperationQueue:
    """One-operation-at-a-time GATT executor with token fencing and priorities."""

    def __init__(self, is_current: Callable[[int], bool]) -> None:
        self._is_current = is_current
        self._pending: list[_QueuedGattOperation[object]] = []
        self._sequence = 0
        self._runner: asyncio.Task[None] | None = None
        self._active: _QueuedGattOperation[object] | None = None
        self._active_task: asyncio.Task[object] | None = None

    async def submit(
        self,
        token: int | None,
        operation: Callable[[], Awaitable[T]],
        *,
        priority: GattPriority = GattPriority.CRITICAL,
        name: str = "gatt",
    ) -> T:
        if token is not None and not self._is_current(token):
            raise asyncio.CancelledError("stale BLE connection token")
        loop = asyncio.get_running_loop()
        future: asyncio.Future[T] = loop.create_future()
        self._sequence += 1
        queued = _QueuedGattOperation(
            int(priority), self._sequence, token, name, operation, future
        )
        heapq.heappush(self._pending, queued)
        if self._runner is None or self._runner.done():
            self._runner = loop.create_task(self._run())
        return await future

    def cancel_stale(self, current_token: int) -> None:
        retained: list[_QueuedGattOperation[object]] = []
        for queued in self._pending:
            if queued.token is not None and queued.token != current_token:
                if not queued.future.done():
                    queued.future.cancel("stale BLE connection token")
            else:
                retained.append(queued)
        self._pending = retained
        heapq.heapify(self._pending)
        if (
            self._active is not None
            and self._active.token is not None
            and self._active.token != current_token
            and self._active_task is not None
            and not self._active_task.done()
        ):
            self._active_task.cancel("BLE connection changed during GATT operation")

    def diagnostic_snapshot(self) -> dict[str, object]:
        """Return read-only queue state for the opt-in host flight recorder."""
        active = self._active
        return {
            "pending_count": len(self._pending),
            "active_name": active.name if active is not None else None,
            "active_sequence": active.sequence if active is not None else None,
            "runner_active": self._runner is not None and not self._runner.done(),
        }

    async def read(
        self, token: int, client: object, characteristic: object, *,
        priority: GattPriority, name: str,
    ) -> object:
        return await self.submit(
            token,
            lambda: client.read_gatt_char(characteristic),  # type: ignore[attr-defined]
            priority=priority,
            name=name,
        )

    async def write(
        self, token: int, client: object, characteristic: object, data: bytes, *,
        response: bool, priority: GattPriority, name: str, timeout: float | None = None,
    ) -> object:
        async def operation() -> object:
            write = client.write_gatt_char(  # type: ignore[attr-defined]
                characteristic, data, response=response
            )
            # Bound the native operation too, so an OTA write timeout cannot
            # leave the serialized executor occupied after its caller exits.
            if timeout is not None:
                return await asyncio.wait_for(write, timeout=timeout)
            return await write

        return await self.submit(
            token,
            operation,
            priority=priority,
            name=name,
        )

    async def write_with_fallback(
        self, token: int, client: object, characteristic: object, data: bytes, *,
        priority: GattPriority, name: str,
        guard: Callable[[], bool] | None = None,
    ) -> bool:
        async def operation() -> bool:
            if guard is not None and not guard():
                raise asyncio.CancelledError("obsolete GATT transaction")
            try:
                await client.write_gatt_char(  # type: ignore[attr-defined]
                    characteristic, data, response=True
                )
                return True
            except Exception:
                if guard is not None and not guard():
                    raise asyncio.CancelledError("obsolete GATT fallback")
                await client.write_gatt_char(  # type: ignore[attr-defined]
                    characteristic, data, response=False
                )
                return False

        return await self.submit(
            token, operation, priority=priority, name=name
        )

    async def start_notify(
        self, token: int, client: object, characteristic: object, callback: object, *,
        priority: GattPriority, name: str,
    ) -> object:
        return await self.submit(
            token,
            lambda: client.start_notify(characteristic, callback),  # type: ignore[attr-defined]
            priority=priority,
            name=name,
        )

    async def stop_notify(
        self, token: int | None, client: object, characteristic: object, *,
        priority: GattPriority, name: str,
    ) -> object:
        return await self.submit(
            token,
            lambda: client.stop_notify(characteristic),  # type: ignore[attr-defined]
            priority=priority,
            name=name,
        )

    async def _run(self) -> None:
        while self._pending:
            queued = heapq.heappop(self._pending)
            if queued.future.done():
                continue
            if queued.token is not None and not self._is_current(queued.token):
                queued.future.cancel("stale BLE connection token")
                continue
            self._active = queued
            self._active_task = asyncio.create_task(queued.operation())
            try:
                result = await self._active_task
                if queued.token is not None and not self._is_current(queued.token):
                    queued.future.cancel("BLE connection changed during GATT operation")
                elif not queued.future.done():
                    queued.future.set_result(result)
            except asyncio.CancelledError as exc:
                if not queued.future.done():
                    queued.future.cancel(str(exc))
            except Exception as exc:
                if not queued.future.done():
                    queued.future.set_exception(exc)
            finally:
                self._active = None
                self._active_task = None
