# Source Snapshot Update Log

## 2026-09-24 — Initial private online stress snapshot

- Firmware: ZY100_Online_Stress_Test 20007 / 1.0.0.2, NO_MOS, 404 KiB APP layout.
- Host: BLE Tool v1.1.108.
- Source provenance: workspace HEAD b1807ad0aa00915bc0b98158de9c8fa67b1b5006 with uncommitted sources; all 422 firmware files match the saved 20007 build dependency hashes. HEAD alone cannot reproduce this snapshot.
- Changes: created separate application, SDK and host trees; added protocol schema, analysis tools, English reading guides and original user-provided logs.
- Key points: preserve stress-specific page retry/readback ownership behavior and the host END-finalization race handling present in the selected source; do not alter algorithms, protocols, source comments or version identifiers.
- Scope: reading-only source handoff; no Keil projects, binary libraries, firmware images, packaged applications, environments or unrelated test suites.
- Static checks: source and log byte hashes, source inventory, reading links, internal Python import targets and common credential patterns.
- Validation boundary: static publication checks only; no independent compilation, software regression run or board validation. Log inclusion is not a stress-test pass.
- Future maintenance: append an English entry for every subsequent snapshot update and regenerate the file inventory.

## 2026-09-24 — Additional original logs

- Added COM3_2026-09-24_18-39-37_ARM.log and ble_tool_logs2.txt under logs/2026-09-24/.
- Preserved original bytes and checked source/destination SHA-256 before and after copying; recorded sizes and hashes in the log index and snapshot manifest.
- Common credential-pattern scan returned no matches. This is not a guarantee that all sensitive information is absent.
- Firmware and host source snapshots are unchanged. No compilation, device test or stress-test acceptance is claimed.
