"""Project-local Bleak 3.0.1 WinRT backend with lossless discovery diagnostics.

No global patching: only clients explicitly selecting this backend use it.
"""
import uuid

from bleak.backends.winrt.client import BleakClientWinRT, FutureLike, _ensure_success
from bleak.backends.service import BleakGATTService, BleakGATTServiceCollection
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.descriptor import BleakGATTDescriptor
from bleak.assigned_numbers import gatt_char_props_to_strs
from winrt.windows.devices.bluetooth import BluetoothCacheMode
from winrt.windows.devices.bluetooth.genericattributeprofile import GattCommunicationStatus
from winrt.windows.devices.enumeration import DeviceAccessStatus

from .connection_recovery import ConnectionFailure


class DiagnosticWinRTClient(BleakClientWinRT):
    def __init__(self, *args, **kwargs):
        self._diagnostic_log = kwargs.pop("diagnostic_log", lambda message: None)
        self._required_gatt = kwargs.pop("required_gatt", {})
        self._bootstrap_services = kwargs.pop("bootstrap_services", set())
        self._owned_raw_services = []
        super().__init__(*args, **kwargs)

    def _record(self, phase, result=None, **fields):
        if result is not None:
            fields.update(status=str(result.status), protocol_error=getattr(result, "protocol_error", None))
        self._diagnostic_log(f"[WINRT_GATT] phase={phase} {fields}")

    async def _service_result(self, service_uuid=None):
        if service_uuid is None:
            operation = self._requester.get_gatt_services_with_cache_mode_async(BluetoothCacheMode.UNCACHED)
        else:
            operation = self._requester.get_gatt_services_for_uuid_with_cache_mode_async(
                uuid.UUID(service_uuid), BluetoothCacheMode.UNCACHED)
        result = await FutureLike(operation)
        self._record("services", result, uuid=service_uuid, cache="uncached")
        if result.status == GattCommunicationStatus.ACCESS_DENIED:
            raise ConnectionFailure("access_denied", "Windows 拒绝 GATT 服务发现访问")
        services = list(_ensure_success(result, "services", "GATT service discovery failed"))
        self._owned_raw_services.extend(services)
        for service in services:
            self._record("raw_service_returned", uuid=str(service.uuid), handle=service.attribute_handle,
                         discovery_mode="all_services" if service_uuid is None else "required_uuid_supplement")
        return services

    async def _add_service(self, service, collection, mode):
        service_uuid = str(service.uuid).lower()
        required = service_uuid in self._required_gatt
        self._record("raw_service", uuid=service_uuid, handle=service.attribute_handle, discovery_mode=mode)
        result = await FutureLike(service.get_characteristics_with_cache_mode_async(BluetoothCacheMode.UNCACHED))
        self._record("characteristics", result, uuid=service_uuid, cache="uncached")
        if result.status == GattCommunicationStatus.ACCESS_DENIED and required:
            access = await FutureLike(service.request_access_async())
            self._record("request_access", uuid=service_uuid, access=str(access))
            if access == DeviceAccessStatus.ALLOWED:
                result = await FutureLike(service.get_characteristics_with_cache_mode_async(BluetoothCacheMode.UNCACHED))
                self._record("characteristics_retry", result, uuid=service_uuid, cache="uncached")
        if result.status == GattCommunicationStatus.ACCESS_DENIED:
            if required:
                raise ConnectionFailure("access_denied", f"私有服务存在但 Windows 拒绝访问：{service_uuid}")
            self._record("skipped_access_denied", uuid=service_uuid)
            return
        chars = _ensure_success(result, "characteristics", f"GATT characteristics failed: {service_uuid}")
        wrapped = BleakGATTService(service, service.attribute_handle, service_uuid)
        collection.add_service(wrapped)
        for char in chars:
            properties = list(gatt_char_props_to_strs(char.characteristic_properties))
            converted = BleakGATTCharacteristic(char, char.attribute_handle, str(char.uuid),
                                               properties, lambda: self.mtu_size - 3, wrapped)
            collection.add_characteristic(converted)
            result = await FutureLike(char.get_descriptors_with_cache_mode_async(BluetoothCacheMode.UNCACHED))
            self._record("descriptors", result, uuid=str(char.uuid), cache="uncached")
            if result.status == GattCommunicationStatus.ACCESS_DENIED:
                raise ConnectionFailure("access_denied", f"Windows 拒绝特征描述符访问：{char.uuid}")
            for descriptor in _ensure_success(result, "descriptors", f"GATT descriptors failed: {char.uuid}"):
                collection.add_descriptor(BleakGATTDescriptor(
                    descriptor, descriptor.attribute_handle, str(descriptor.uuid), converted))

    async def _get_services(self, *, service_cache_mode=None, cache_mode=None, **kwargs):
        if self.services is not None:
            return self.services
        # Rebuild only from this discovery attempt. Never import old handles.
        collection = BleakGATTServiceCollection()
        services = await self._service_result()
        # Critical services first; keep unrelated discovery off the bootstrap path.
        services.sort(key=lambda s: str(s.uuid).lower() not in self._required_gatt)
        for service in services:
            if self._required_gatt and str(service.uuid).lower() not in (set(self._required_gatt) | self._bootstrap_services):
                self._record("raw_service_deferred", uuid=str(service.uuid), handle=service.attribute_handle)
                continue
            await self._add_service(service, collection, "all_services")
        for service_uuid, characteristic_uuids in self._required_gatt.items():
            present = collection.get_service(service_uuid)
            found = {c.uuid.lower() for c in present.characteristics} if present else set()
            if present and set(characteristic_uuids).issubset(found):
                continue
            # One targeted supplement per required UUID, within the same attempt.
            replacements = await self._service_result(service_uuid)
            if present:
                # Replace this service as one unit, not a mixture of handles.
                rebuilt = BleakGATTServiceCollection()
                for existing in collection:
                    if existing.uuid.lower() == service_uuid:
                        continue
                    rebuilt.add_service(existing)
                    for char in existing.characteristics:
                        rebuilt.characteristics[char.handle] = char
                        for descriptor in char.descriptors:
                            rebuilt.descriptors[descriptor.handle] = descriptor
                collection = rebuilt
            for service in replacements:
                await self._add_service(service, collection, "required_uuid_supplement")
        # Release unused, denied and superseded raw services now, not at GC time.
        retained = {id(s.obj) for s in collection}
        unused = [s for s in self._owned_raw_services if id(s) not in retained]
        failures = []
        for service in unused:
            try:
                service.close()
            except Exception as exc:
                failures.append(service)
                self._record("close_failed", uuid=str(service.uuid), error=str(exc))
        self._owned_raw_services = failures
        if failures:
            # Keep the retained objects owned as well if connect cannot complete.
            self._owned_raw_services.extend(s.obj for s in collection)
            raise ConnectionFailure("cleanup", "GATT 临时资源释放失败")
        self._record("usable_services", count=len(list(collection)), cache="uncached")
        return collection
