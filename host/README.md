# BLE Tool Source Reading Guide

Identity: BLE Tool v1.1.108, as declared in [ui/main_window.py](ui/main_window.py).

Read [main.py](main.py), then the UI and [BLE manager](ble/ble_manager.py). The main module performs incomplete-session recovery before starting the interface; do not execute it merely to inspect this snapshot.

- [Stress controller](ble/online_stress_controller.py): probe, configuration, capability negotiation, start/stop and session state.
- [Stress protocol](ble/online_stress.py): decoding and deterministic payload validation.
- [Storage worker](ble/online_storage_worker.py) and [online store](ble/online_store.py): asynchronous persistence and completion handling.
- [Stress report](ble/online_stress_report.py): integrity and diagnostic reporting.
- [Stress UI](ui/online_stress_view.py): controls and displayed results.
- [Dependencies](requirements.txt) and [GATT reference](gatt_services.txt).

The original main, ble, ui and ota module structure is retained. Offline, calibration and OTA modules are included because the common host imports them; this is not a new stripped-down application. host_selftest.py is an imported runtime module, not an unrelated copied test suite.

Runtime context: Windows BLE support, Python with Tkinter, and bleak 3.0.1 (plus its resolved platform dependencies). Environments, installers, bundled firmware and workstation settings are excluded. No standalone execution or packaging guarantee is made.
