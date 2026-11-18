"""Run the mock controller: python -m tools.elfin_mock [--cmd-port 10003] [--state-port 10004] ...

Point a tool at it with --mock (the tools default to 127.0.0.1 and these ports).
The mock tests our framing and our reactions, not the arm's behaviour (codebase plan §3.8 item 10).
"""
import argparse
import time

from tools.elfin_mock.server import MockServer
from tools.elfin_mock.sim import MockArm, MockConfig


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.elfin_mock", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--cmd-port", type=int, default=10003)
    ap.add_argument("--state-port", type=int, default=10004)
    ap.add_argument("--unpowered", action="store_true", help="start with nElectrify = 0")
    ap.add_argument("--error", type=int, default=0, help="start in the error state with this code")
    ap.add_argument("--servo-moving-state", choices=["tracking", "always1", "always0"], default="tracking")
    ap.add_argument("--on-push-starvation", choices=["hold", "error"], default="hold")
    ap.add_argument("--byte-order", choices=["<", ">"], default="<", help="Datasheet length fields")
    ap.add_argument("--fail", action="append", default=[], metavar="NAME:N:CODE",
                    help="reply Fail with CODE to the N-th NAME command (repeatable)")
    ap.add_argument("--late", action="append", default=[], metavar="NAME:N:SECONDS",
                    help="delay the reply to the N-th NAME command (repeatable)")
    ap.add_argument("--drop-after", type=int, default=0, help="close the connection after N messages")
    ap.add_argument("--no-fragment", action="store_true")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--mirror-config", metavar="E03PRO_YAML",
                    help="report this file's limits, frames and joint ranges (a rehearsal of an arm run)")
    a = ap.parse_args(argv)

    mirror = {}
    if a.mirror_config:
        from tools import yamlmini
        from tools.elfin_mock.sim import mirror_config
        mirror = mirror_config(yamlmini.load(a.mirror_config))
    cfg = MockConfig(**mirror, powered=not a.unpowered, servo_moving_state=a.servo_moving_state,
                     on_push_starvation=a.on_push_starvation, datasheet_byte_order=a.byte_order,
                     drop_after_messages=a.drop_after, fragment_replies=not a.no_fragment, seed=a.seed,
                     fail_rules=[(n, int(k), int(c)) for n, k, c in (s.split(":") for s in a.fail)],
                     late_reply_rules=[(n, int(k), float(d)) for n, k, d in (s.split(":") for s in a.late)])
    arm = MockArm(cfg)
    if a.error:
        arm.error_code = a.error
    with MockServer(arm, a.host, a.cmd_port, a.state_port) as srv:
        print(f"mock Elfin controller on {a.host}: commands {srv.cmd_port}, Datasheet {srv.state_port}. Ctrl-C to stop.",
              flush=True)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        st = arm.stats
        print(f"messages {st['messages']}, format errors {st['format_errors']}, forbidden {st['forbidden']}, "
              f"dropped extra messages {st['dropped_extra_messages']}, pushes {st['pushes']}")


if __name__ == "__main__":
    main()
