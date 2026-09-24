"""Best-effort exhaustive teardown, shared by application and DFU clients."""
import asyncio


class ClientCleanupError(RuntimeError):
    pass


def close_backend(client, log=lambda message: None) -> bool:
    backend = getattr(client, "_backend", None)
    if backend is None:
        return True
    errors = []

    def perform(label, action):
        try:
            action()
        except Exception as exc:
            errors.append(label)
            log(f"[BLE_CLEANUP] {label}: {type(exc).__name__}: {exc}")

    session = getattr(backend, "_session", None)
    requester = getattr(backend, "_requester", None)
    if session is not None:
        perform("maintain_connection", lambda: setattr(session, "maintain_connection", False))
    services = getattr(backend, "services", None)
    callbacks = getattr(backend, "_notification_callbacks", {})
    for handle, token in list(callbacks.items()):
        def remove_notify(handle=handle, token=token):
            char = services.get_characteristic(handle) if services is not None else None
            if char is not None:
                char.obj.remove_value_changed(token)
        perform("notification", remove_notify)
    callbacks.clear()
    for owner, token_attr, method in (
        (session, "_session_status_changed_token", "remove_session_status_changed"),
        (session, "_max_pdu_size_changed_token", "remove_max_pdu_size_changed"),
        (requester, "_services_changed_token", "remove_gatt_services_changed"),
    ):
        token = getattr(backend, token_attr, None)
        if owner is not None and token is not None:
            perform(token_attr, lambda owner=owner, method=method, token=token: getattr(owner, method)(token))
            setattr(backend, token_attr, None)
    raw = list(getattr(backend, "_owned_raw_services", []))
    if services is not None:
        raw.extend(s.obj for s in services)
    seen = set()
    for obj in raw:
        if id(obj) not in seen:
            seen.add(id(obj))
            perform("service.close", obj.close)
    backend.services = None
    if hasattr(backend, "_owned_raw_services"):
        backend._owned_raw_services.clear()
    for attr, obj in (("_session", session), ("_requester", requester)):
        if obj is not None:
            perform(attr + ".close", obj.close)
            setattr(backend, attr, None)
    failed = bool(errors) or bool(getattr(backend, "_cleanup_failed", False))
    backend._cleanup_failed = failed
    log(f"[BLE_CLEANUP] complete={int(not failed)} services={len(seen)} system_link_not_owned=1")
    return not failed


async def release_client(client, log=lambda message: None) -> None:
    cancelled = False
    try:
        # A remote disconnect does not prove that WinRT objects were released.
        await asyncio.wait_for(client.disconnect(), timeout=5.0)
    except asyncio.CancelledError:
        cancelled = True
    except Exception as exc:
        log(f"[BLE_CLEANUP] disconnect: {type(exc).__name__}: {exc}; forcing teardown")
    finally:
        clean = close_backend(client, log)
    if not clean:
        raise ClientCleanupError("本应用 BLE 资源释放失败；未启动下一次连接")
    if cancelled:
        raise asyncio.CancelledError
