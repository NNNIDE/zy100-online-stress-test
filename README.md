# ZY100 Online Stress Test Source Snapshot

Private source-reading handoff dated 2026-09-24.

| Component | Identity |
| --- | --- |
| Firmware | ZY100 Online Stress Test, 20007 / 1.0.0.2 |
| Hardware configuration | ZY100 V1.2, NO_MOS |
| APP partition | 404 KiB (413696 bytes), not the Production 10607 historical layout |
| Host | BLE Tool v1.1.108 |

## Reading map

- [Firmware](firmware/README.md): 422 build-dependency C/header/assembly files, split into application and SDK trees.
- [Host](host/README.md): original Python modules and runtime dependency references.
- [Architecture](docs/ARCHITECTURE.md): startup, generation, Flash, BLE, persistence and ACK flow.
- [Update log](docs/UPDATE_LOG.md): snapshot history and validation boundaries.
- [File inventory](docs/SNAPSHOT_MANIFEST.json): original-byte sizes and SHA-256.
- [Captured logs](logs/2026-09-24/README.md): four unchanged user-supplied logs.

This is a reading snapshot, not an independently buildable or runnable distribution. Keil projects, proprietary binary libraries, firmware images, executables, installers, environments, private settings, build outputs and unrelated tests are intentionally absent. Original source comments and logs are not translated. No algorithm, protocol or firmware version was modified.

Firmware provenance is the 20007 build dependency hash inventory, not a clean export of the workspace HEAD. Required shared modules remain even where their names mention Production, Factory gates or diagnostics. Host core modules retain their original structure, including offline/calibration/OTA dependencies needed to understand the complete UI.

## Evidence boundary

All 422 firmware source hashes match the saved build dependency inventory. Host and log bytes were checked before and after copying. These are static snapshot checks, not new compilation, executable reproducibility, device testing or stress-test acceptance. The saved firmware manifest reports verified build / pending board test. Merely including the logs does not establish a pass or a sustainable throughput limit.

Future updates should replace this reading snapshot only after identity and hash checks, then append an English entry to the update log. This repository is independent of the original development workspaces and liwofeng/zhipai.
