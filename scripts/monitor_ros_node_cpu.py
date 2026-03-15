#!/usr/bin/env python3
"""
Monitor CPU usage of running ROS 2 node processes.

Outputs per-process CPU percent and share of the total observed ROS-node CPU.
"""

import argparse
import csv
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import psutil


@dataclass
class NodeProcess:
    pid: int
    node_name: str
    exec_name: str
    cmdline: str
    proc: psutil.Process
    primed: bool = False


def parse_args():
    parser = argparse.ArgumentParser(
        description="Sample CPU usage of running ROS 2 node processes.")
    parser.add_argument(
        "--interval", type=float, default=1.0,
        help="Sampling interval in seconds. Default: 1.0")
    parser.add_argument(
        "--top", type=int, default=20,
        help="Maximum number of rows to print. Default: 20")
    parser.add_argument(
        "--once", action="store_true",
        help="Sample once and exit.")
    parser.add_argument(
        "--include", action="append", default=[],
        help="Only include nodes whose name, executable, or command contains this substring. Can be repeated.")
    parser.add_argument(
        "--csv", type=str, default="",
        help="Optional CSV file path for appending samples.")
    parser.add_argument(
        "--no-clear", action="store_true",
        help="Do not clear the terminal between refreshes.")
    return parser.parse_args()


def extract_node_name(cmdline):
    for idx, arg in enumerate(cmdline):
        if arg.startswith("__node:="):
            return arg.split(":=", 1)[1]

        if arg in ("-r", "--remap") and idx + 1 < len(cmdline):
            remap = cmdline[idx + 1]
            if remap.startswith("__node:="):
                return remap.split(":=", 1)[1]

        if "__node:=" in arg:
            return arg.split("__node:=", 1)[1]

    return None


def extract_exec_name(cmdline):
    if not cmdline:
        return "unknown"

    exe = Path(cmdline[0]).name
    if exe.startswith("python") and len(cmdline) > 1:
        return Path(cmdline[1]).name
    return exe


def is_ros_node_process(cmdline):
    if not cmdline:
        return False

    exe = Path(cmdline[0]).name
    if exe == "ros2" and "launch" in cmdline[1:3]:
        return False

    if "--ros-args" in cmdline:
        return True

    return any("__node:=" in arg for arg in cmdline)


def matches_filters(node_name, exec_name, cmdline, include_filters):
    if not include_filters:
        return True

    haystack = " ".join((node_name, exec_name, cmdline)).lower()
    return any(filt.lower() in haystack for filt in include_filters)


def discover_node_processes(include_filters):
    found = {}

    for proc in psutil.process_iter(["pid", "cmdline"]):
        try:
            cmdline = proc.info.get("cmdline") or []
            if not is_ros_node_process(cmdline):
                continue

            node_name = extract_node_name(cmdline)
            exec_name = extract_exec_name(cmdline)
            if node_name is None:
                node_name = exec_name

            cmdline_str = " ".join(cmdline)
            if not matches_filters(node_name, exec_name, cmdline_str, include_filters):
                continue

            found[proc.pid] = NodeProcess(
                pid=proc.pid,
                node_name=node_name,
                exec_name=exec_name,
                cmdline=cmdline_str,
                proc=psutil.Process(proc.pid),
            )
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    return found


def sync_processes(tracked, include_filters):
    discovered = discover_node_processes(include_filters)

    for pid in list(tracked):
        if pid not in discovered:
            tracked.pop(pid, None)

    for pid, discovered_proc in discovered.items():
        if pid in tracked:
            tracked_proc = tracked[pid]
            tracked_proc.node_name = discovered_proc.node_name
            tracked_proc.exec_name = discovered_proc.exec_name
            tracked_proc.cmdline = discovered_proc.cmdline
        else:
            tracked[pid] = discovered_proc
            try:
                tracked[pid].proc.cpu_percent(None)
                tracked[pid].primed = True
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                tracked.pop(pid, None)

    return tracked


def take_sample(tracked):
    rows = []
    total_cpu = 0.0

    for pid, node_proc in list(tracked.items()):
        try:
            cpu = node_proc.proc.cpu_percent(None)
            rss_mb = node_proc.proc.memory_info().rss / (1024 * 1024)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            tracked.pop(pid, None)
            continue

        rows.append({
            "node": node_proc.node_name,
            "pid": node_proc.pid,
            "exec": node_proc.exec_name,
            "cpu": cpu,
            "rss_mb": rss_mb,
            "cmdline": node_proc.cmdline,
        })
        total_cpu += cpu

    for row in rows:
        row["share"] = (row["cpu"] / total_cpu * 100.0) if total_cpu > 0.0 else 0.0

    rows.sort(key=lambda row: row["cpu"], reverse=True)
    return rows, total_cpu


def truncate(text, width):
    if len(text) <= width:
        return text
    if width <= 1:
        return text[:width]
    return text[:width - 1] + "…"


def print_table(rows, total_cpu, interval, top, clear_screen):
    if clear_screen:
        print("\033[2J\033[H", end="")

    print(f"ROS node CPU monitor  interval={interval:.2f}s")
    print("CPU% is per-process usage and can exceed 100 on multicore systems.")
    print(f"Observed ROS-node CPU total: {total_cpu:.1f}% across {len(rows)} processes")
    print("")

    if not rows:
        print("No matching ROS node processes found.")
        return

    term_width = shutil.get_terminal_size((140, 20)).columns
    node_w = 28
    pid_w = 7
    cpu_w = 8
    share_w = 9
    rss_w = 9
    exec_w = max(18, min(28, term_width - (node_w + pid_w + cpu_w + share_w + rss_w + 10)))

    header = (
        f"{'NODE':<{node_w}} "
        f"{'PID':>{pid_w}} "
        f"{'CPU%':>{cpu_w}} "
        f"{'SHARE%':>{share_w}} "
        f"{'RSS_MB':>{rss_w}} "
        f"{'EXEC':<{exec_w}}"
    )
    print(header)
    print("-" * min(len(header), term_width))

    for row in rows[:top]:
        print(
            f"{truncate(row['node'], node_w):<{node_w}} "
            f"{row['pid']:>{pid_w}d} "
            f"{row['cpu']:>{cpu_w}.1f} "
            f"{row['share']:>{share_w}.1f} "
            f"{row['rss_mb']:>{rss_w}.1f} "
            f"{truncate(row['exec'], exec_w):<{exec_w}}"
        )


def append_csv(csv_path, rows, total_cpu):
    path = Path(csv_path)
    write_header = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow([
                "timestamp",
                "node",
                "pid",
                "exec",
                "cpu_percent",
                "ros_cpu_share_percent",
                "rss_mb",
                "ros_total_cpu_percent",
                "cmdline",
            ])

        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        for row in rows:
            writer.writerow([
                timestamp,
                row["node"],
                row["pid"],
                row["exec"],
                f"{row['cpu']:.3f}",
                f"{row['share']:.3f}",
                f"{row['rss_mb']:.3f}",
                f"{total_cpu:.3f}",
                row["cmdline"],
            ])


def main():
    args = parse_args()
    if args.interval <= 0.0:
        print("--interval must be > 0", file=sys.stderr)
        return 2

    tracked = {}
    sync_processes(tracked, args.include)

    try:
        while True:
            time.sleep(args.interval)
            sync_processes(tracked, args.include)
            rows, total_cpu = take_sample(tracked)
            print_table(
                rows=rows,
                total_cpu=total_cpu,
                interval=args.interval,
                top=args.top,
                clear_screen=(sys.stdout.isatty() and not args.no_clear and not args.once),
            )

            if args.csv:
                append_csv(args.csv, rows, total_cpu)

            if args.once:
                break
    except KeyboardInterrupt:
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
