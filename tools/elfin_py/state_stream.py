"""Read the state stream (example 4, extended): the Datasheet on port 10004.

    python -m tools.elfin_py.state_stream --dry-run | --mock | --arm [--seconds 10]

Reports the packet spacing (nominally 50 ms at 20 Hz, elfin_commands.md, PROTO p.7), the byte
order of the length fields, whether the manual's quirks are present (string numbers, a
trailing comma; PROTO p.185-186), and every key the real packets carry. The last answers
codebase plan §3.8 item 7: whether the packets carry an e-stop or light-curtain flag.
Port 10003 is not opened. Force-sensor fields are neither printed nor used (owner, week 1).
"""
import statistics
import sys
import time

from tools.elfin_py import cli, link, triallog

FIRST_RUN = "Packet rate and format; nothing is sent to the arm."
SAFETY_HINTS = ("emerg", "esto", "estop", "safety", "guard", "curtain")


def main(argv=None):
    ap = cli.parser(__doc__)
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args(argv)
    with cli.session(args, "state_stream", "scratch", FIRST_RUN, connect=False) as (arm, log, *_):
        stream = link.StateStream(arm, log).start()
        t_end = time.monotonic() + args.seconds
        shown = 0
        while time.monotonic() < t_end and stream.error is None:
            time.sleep(0.05)
            if len(stream.packets) >= shown + 20:
                shown = len(stream.packets)
                _, p = stream.packets[-1]
                q = [round(x * 57.29577951308232, 3) for x in p["q_rad"]] if p["q_rad"] else None
                print(f"J (deg) {q}  moving {p['state']['robotMoving']}  enabled {p['state']['robotEnabled']}")
        stream.stop()
        if stream.error:
            print(f"stream error: {stream.error!r}", file=sys.stderr)
        arrivals = [a for a, _ in stream.packets]
        gaps_ms = [(b - a) / 1e6 for a, b in zip(arrivals, arrivals[1:])]
        keys = sorted(stream.keys)
        safety_keys = [k for k in keys if any(h in k.lower() for h in SAFETY_HINTS)]
        summary = {
            "packets": len(arrivals),
            "duration_s": (arrivals[-1] - arrivals[0]) / 1e9 if len(arrivals) > 1 else 0.0,
            "gap_mean_ms": statistics.fmean(gaps_ms) if gaps_ms else None,
            "gap_max_ms": max(gaps_ms) if gaps_ms else None,
            "byte_orders": sorted(stream.byte_orders),
            "keys": keys,
            "safety_keys": safety_keys,
            "error": repr(stream.error) if stream.error else None,
        }
        log.event(triallog.EV_SUMMARY, summary)
        print(f"\n{summary['packets']} packets in {summary['duration_s']:.1f} s; gap mean "
              f"{summary['gap_mean_ms']} ms, max {summary['gap_max_ms']} ms; byte order {summary['byte_orders']}")
        print("keys:\n  " + "\n  ".join(keys))
        print("e-stop / light-curtain keys: " + (", ".join(safety_keys) if safety_keys else
              "NONE (codebase plan §3.8 item 7: the ladder must then test ReadRobotState between pushes)"))
        return 0 if arrivals and not stream.error else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        sys.exit(f"stopped: {exc}")
