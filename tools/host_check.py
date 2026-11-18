"""Stage 0.1: is this the machine for arm runs and the RT process? (codebase plan §2, §6, §8)

    python -m tools.host_check [--data-dir DIR] [--result-json PATH]

Standard library only. Reports; changes nothing. The real-time settings (PREEMPT_RT, isolated cores)
are shown for the owner to judge, because their week-1 choices are not written down yet [unverified].
"""
import argparse
import json
import os
import pathlib
import platform
import shutil
import sys
import time

from tools.elfin_py import link

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _read(path):
    try:
        return pathlib.Path(path).read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return None


def check(data_dir):
    data_dir = pathlib.Path(data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    problem = link.arm_host_problem()
    info = time.get_clock_info("monotonic")
    cmdline = _read("/proc/cmdline") or ""
    m = {
        "platform": platform.platform(), "python": sys.version.split()[0],
        "python_ok": sys.version_info >= (3, 12),
        "arm_host_ok": problem is None, "arm_host_problem": problem,
        "monotonic_resolution_s": info.resolution, "monotonic_impl": info.implementation,
        "data_dir": str(data_dir), "data_free_gb": round(shutil.disk_usage(data_dir).free / 1e9, 1),
        "preempt_rt": _read("/sys/kernel/realtime") == "1" or "PREEMPT_RT" in (platform.version() or ""),
        "isolcpus": next((w for w in cmdline.split() if w.startswith("isolcpus=")), None),
        "cpus": os.cpu_count(),
        "backup_disk": os.environ.get("BACKUP_DISK"),
    }
    return m


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", default=os.environ.get("SLOSH_DATA", str(ROOT / "data")))
    ap.add_argument("--result-json")
    a = ap.parse_args(argv)
    m = check(a.data_dir)
    for k, v in m.items():
        print(f"{k:24s} {v}")
    if a.result_json:
        pathlib.Path(a.result_json).write_text(json.dumps(m, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
