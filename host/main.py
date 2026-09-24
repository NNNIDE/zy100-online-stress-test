from __future__ import annotations

import argparse
import asyncio
import os
import tkinter as tk

from ble.host_selftest import run_host_ble_self_check_offline
from ble.link_trace import (
    TRACE_DIR_ENV,
    TRACE_ENV,
    configure_bleak_debug_logging,
)
from ui.main_window import MainWindow
from ble.online_store import OnlineSessionStore


def main() -> None:
    parser = argparse.ArgumentParser(description="ZY100 BLE Tool")
    parser.add_argument("--ble-self-check", action="store_true", help="run BLE host self-check and exit")
    parser.add_argument(
        "--ble-link-trace",
        action="store_true",
        help="enable the in-memory BLE supervision-timeout flight recorder",
    )
    parser.add_argument(
        "--ble-link-trace-dir",
        default="diagnostics",
        help="directory for diagnostic JSONL and rotating Bleak logs",
    )
    args = parser.parse_args()
    if args.ble_link_trace:
        os.environ[TRACE_ENV] = "1"
        os.environ[TRACE_DIR_ENV] = args.ble_link_trace_dir
        configure_bleak_debug_logging(True, args.ble_link_trace_dir)
    if args.ble_self_check:
        asyncio.run(run_host_ble_self_check_offline(log=print, scan=True, write_report=True))
        return

    root = tk.Tk()
    # Once, before BLE reception. Browser/store construction must stay read-only.
    try:
        OnlineSessionStore().recover_incomplete_sessions()
    except (OSError, ValueError) as exc:
        from tkinter import messagebox
        messagebox.showerror("Online storage recovery failed", str(exc), parent=root)
        root.destroy()
        return
    MainWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
