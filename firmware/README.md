# Firmware Reading Guide

Target: ZY100_Online_Stress_Test, 20007 / 1.0.0.2, ZY100 V1.2, NO_MOS. The APP partition is 404 KiB.

Application and SDK paths preserve their original paths relative to their respective roots. The dependency closure includes only saved-build source files; compiler headers and binary SDK libraries are excluded. The protocol JSON and two analysis scripts are additional reading material, not part of the 422-file count.

Start here:

1. [Private version](zy100-app/online_stress_test/config/version.h) and [Flash map](zy100-app/flash_map.h).
2. [main.c](zy100-app/src/sample/ble_peripheral/main.c), main(): board/platform setup and application startup.
3. [app_task.c](zy100-app/src/sample/ble_peripheral/app_task.c), app_task_init() and app_main_task(): task creation and event-driven orchestration.
4. [Stress controller](zy100-app/online_stress_test/src/zy100_online_stress.c): configuration and session lifecycle.
5. [Stress generator](zy100-app/online_stress_test/src/zy100_stress_source.c): deterministic load and ring accounting.
6. [Protocol schema](zy100-app/online_stress_test/protocol_v1.json): shared wire-format reference.
7. [Timing analysis](zy100-app/online_stress_test/tools/analyze_timing.py): offline report source; its --host-root option can point to the sibling host snapshot instead of its original workstation default.

The realtek-sdk directory contains the referenced SDK implementation and interfaces, retaining original copyright notices. No license grant for omitted proprietary components is implied. This layout deliberately does not reproduce compiler include paths or a complete build environment.
