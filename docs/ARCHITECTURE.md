# Architecture and Reading Route

## Firmware

The dedicated target shares platform startup, BLE services and application task infrastructure with ZY100, while selecting its private version and stress implementation. Follow main.c into task initialization and the app_main_task event loop. The task coordinates BLE events, sensor acquisition, storage and stress-session work; low-level SDK interfaces remain under firmware/realtek-sdk.

The stress controller negotiates a session and rate. The deterministic generator feeds a 4096-byte ring on a nominal 20 ms period. Configured rates are 0, 16000, 32000, 48000, 64000 and 96000 bytes/second; these are requested loads, not measured throughput guarantees. Normal sensor data and synthetic stress records enter the existing online spool/stream path.

Generation -> ring -> Flash page programming -> readback/commit -> BLE fragments -> host persistence -> ACK

Read the application service modules zy100_online_raw_capture.c, zy100_online_spool.c and zy100_online_stream_v2.c alongside the dedicated stress sources. The ring is not a substitute for durable storage: source release is tied to the storage path. A full ring is an error condition rather than permission to silently reduce the requested load. Disconnection does not imply resumable online capture.

Version 20007 enables stress-specific page-retry behavior: a pending page readback that encounters BUSY can return to service work and be revisited without adding an unconditional RTOS-tick delay. Other readers observing WIP clear must not discard the writer's pending readback. Existing bounded work and periodic scheduling safeguards remain relevant. This snapshot does not change Production behavior or import Production 10607's old APP layout.

## Host

main.py starts the UI and recovery path. main_window.py coordinates the BLE manager and stress view. online_stress_controller.py probes command 0x30, checks the protocol/capability response and configures the requested load before START. Consult protocol_v1.json and online_stress.py for exact field layouts; the JSON alone is not a replacement for parser and state-machine checks.

Received fragments are reassembled and validated through the online protocol/store path. online_storage_worker.py moves persistence work off the BLE orchestration path. Store completion, record checks and session identity govern ACK and terminal handling; a received notification alone is not proof of durable capture.

online_stress_report.py combines stored-record integrity with timing/terminal diagnostics. Version 1.1.108 handles the END-ACK / asynchronous commit_end / WAIT_START race only for the matching session context during finalization; it does not waive persistence or ACK requirements.

## Analysis and limits

The two firmware tools parse saved timing and I/O evidence without device I/O. analyze_timing.py imports the host report module and its adjacent stress_io_report.py. Its original local default path is retained unchanged; use its host-root option when reading or using it elsewhere.

The supplied text and UART logs are original evidence, not a complete capture archive or acceptance certificate. No new build, GUI execution, BLE connection, flashing or board validation was performed for this publication. The saved build identity explicitly records board_validation=not_run.
