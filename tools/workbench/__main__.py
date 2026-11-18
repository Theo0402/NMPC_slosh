"""The workbench (codebase plan §12): python -m tools.workbench [--port 8765] [--data-dir DIR]

Opens a local page at http://127.0.0.1:8765 from which every stage of the hardware phase is run in order.
It is a separate process from the tools and the RT process and is never in the control loop. The stop
button is a software stop; keep the e-stop within reach.
"""
import argparse
import sys

from tools.workbench.backup import default_folder
from tools.workbench.bench import Workbench
from tools.workbench.datalock import LockHeld
from tools.workbench.server import serve


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.workbench", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--data-dir", help="default: slosh/data, or SLOSH_DATA")
    ap.add_argument("--backup-dir", help=f"local backup folder, used until you choose another on the Backup tab "
                                         f"(default {default_folder()})")
    a = ap.parse_args(argv)
    bench = Workbench(data_dir=a.data_dir)
    try:
        lock = bench.claim_data_folder()           # before anything is written (pre-hardware review D6)
    except LockHeld as exc:
        sys.exit(str(exc))
    if lock.taken_over:
        print(f"took over a lock left by a workbench that stopped: {lock.taken_over}")
    try:
        httpd = serve(bench, port=a.port)
    except OSError as exc:
        bench.release_data_folder()
        sys.exit(f"port {a.port} is in use ({exc}): is a workbench already running? Open http://127.0.0.1:{a.port}")
    try:
        _serve(bench, httpd, a)
    finally:
        bench.release_data_folder()


def _serve(bench, httpd, a):
    if a.backup_dir or not bench.backup.settings().get("folder"):
        bench.backup.set_folder(a.backup_dir or default_folder(), bench.backup.settings().get("auto", True))
    rec = bench.recover_runs()
    hp = bench.host_problem()
    print(f"workbench on http://127.0.0.1:{httpd.server_address[1]}  (data: {bench.data_dir})")
    print(f"run folders: {bench.runs_dir}   backup folder: {bench.backup.settings()['folder']}"
          + ("  (backs up after every run)" if bench.backup.settings().get("auto", True) else ""))
    if rec["interrupted"] or rec["reconstructed"]:
        print(f"closed {len(rec['interrupted'])} interrupted run folder(s); made {len(rec['reconstructed'])} folder(s) "
              f"for runs started outside the workbench or before run folders existed")
    for name, err in rec["unreadable"]:
        print(f"SKIPPED {name}: its run.json cannot be read ({err}); the folder was left as it is")
    if rec["still_open"]:
        print(f"left {len(rec['still_open'])} trial log(s) alone: not closed yet, so a tool started outside the "
              f"workbench may still be writing them ({', '.join(rec['still_open'][:3])}"
              f"{', ...' if len(rec['still_open']) > 3 else ''}); a later start-up takes them in")
    print("arm mode: " + ("allowed on this machine" if not hp else f"not available here ({hp})"))
    print("The stop button is a software stop. Keep the e-stop within reach. Ctrl-C ends the workbench;")
    print("a tool still running then stops itself safely (its channel closes).", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if bench.run is not None and bench.run.running:
            print("stopping the running tool ...", file=sys.stderr)
            bench.run.stop()
            bench.run.thread.join(timeout=30)
        httpd.server_close()


if __name__ == "__main__":
    main()
