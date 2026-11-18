"""Enable and move (example 2): J6 by +10 deg, then the tool 20 mm up. With --return, back again.

    python -m tools.elfin_py.move_test --dry-run | --mock | --arm [--return]

Example 2's speeds, checked against confirmed.yaml before sending: joint move 20 deg/s and
100 deg/s^2 (WayPoint nMoveType 0, nIsUseJoint 1), linear move 20 mm/s and 100 mm/s^2
(nMoveType 1) (elfin_commands.md, PROTO p.137-139). The current tool and user frames must
match e03pro.yaml (zero on the bare flange) before the linear move (example 2's check).

The first WayPoint is also the test of the number format: it sends "20.0000" where the
manual's examples show "20" [UNTESTED]. If the arm refuses it, the code is printed; add a
FORMAT_OVERRIDES entry in protocol.py and run again (owner, week 1).
"""
import sys

from tools.elfin_py import cli, link, motion, triallog
from tools.elfin_py.elfin_min import ElfinError

FIRST_RUN = ("Enabling releases the brakes 'with sound and slight movement' (elfin_overview.md, LINUX p.17-18).\n"
             "The first move is J6 +10 deg at 20 deg/s; the second, the flange 20 mm up at 20 mm/s.\n"
             "Hit the e-stop if anything else moves. The first WayPoint tests whether '20.0000' is accepted.")
JOINT_V, JOINT_A, LIN_V, LIN_A = 20.0, 100.0, 20.0, 100.0     # example 2 (elfin_examples.md)


def joint_move(arm, pose, target, m, cmd_id):
    arm.cmd("WayPoint", 0, *pose, *target, m["tcp_name"], m["ucs_name"], JOINT_V, JOINT_A, 0.0, 0, 1, 0, 0, 0, cmd_id)
    arm.wait_done(joints=target)        # done = rest after the move was seen, or the arm at the target (review S12)


def linear_move(arm, pose, joints, m, cmd_id):
    arm.cmd("WayPoint", 0, *pose, *joints, m["tcp_name"], m["ucs_name"], LIN_V, LIN_A, 0.0, 1, 0, 0, 0, 0, cmd_id)
    arm.wait_done(pose=pose)


def check_ranges(arm_cfg, target):
    ranges = arm_cfg.get("joint_ranges_deg")
    if ranges is None:
        print("note: joint_ranges_deg not entered in e03pro.yaml; relying on the controller's own ranges")
        return
    for j, (v, (lo, hi)) in enumerate(zip(target, ranges), 1):
        if not lo <= v <= hi:
            raise link.Refused(f"J{j} target {v:.3f} deg is outside the pendant's range [{lo}, {hi}]")


def main(argv=None):
    ap = cli.parser(__doc__)
    ap.add_argument("--return", dest="ret", action="store_true", help="move back to the start afterwards")
    args = ap.parse_args(argv)
    with cli.session(args, "move_test", "scratch", FIRST_RUN) as (arm, log, confirmed, arm_cfg, ask):
        motion.check_waypoint("joint", JOINT_V, JOINT_A, confirmed)
        motion.check_waypoint("linear", LIN_V, LIN_A, confirmed)
        m = arm_cfg["mounting"]
        with arm.disable_on_exit():
            arm.prepare_and_enable(arm_cfg, ask)
            joints0, pose0, _, _ = arm.act_pos()
            target = joints0[:5] + [joints0[5] + 10.0]
            check_ranges(arm_cfg, target)
            log.event(triallog.EV_PHASE, "joint move J6 +10 deg")
            try:
                joint_move(arm, pose0, target, m, "1")
            except ElfinError as exc:
                print(f"\nThe first WayPoint was refused: {exc}\nIf this is a format error, the controller may not "
                      "accept '20.0000': add a FORMAT_OVERRIDES entry in tools/elfin_py/protocol.py.", file=sys.stderr)
                raise
            arm.check_before_motion()
            joints, pose, tool, user = arm.act_pos()
            problems = link.frame_problems(tool, user, m)
            if problems:
                raise link.Refused("before the linear move: " + "; ".join(problems))
            log.event(triallog.EV_PHASE, "linear move Z +20 mm")
            linear_move(arm, pose[:2] + [pose[2] + 20.0] + pose[3:], joints, m, "2")
            if args.ret:
                arm.check_before_motion()
                j, p, _, _ = arm.act_pos()
                linear_move(arm, p[:2] + [p[2] - 20.0] + p[3:], j, m, "3")
                arm.check_before_motion()
                j, p, _, _ = arm.act_pos()
                joint_move(arm, p, j[:5] + [j[5] - 10.0], m, "4")
            joints1, pose1, _, _ = arm.act_pos()
            summary = {"joints_start": joints0, "pose_start": pose0, "joints_end": joints1, "pose_end": pose1,
                       "format_test": "WayPoint with 4-decimal speeds accepted"}
            log.event(triallog.EV_SUMMARY, summary)
            print(f"joints start {joints0}\njoints end   {joints1}\npose start   {pose0}\npose end     {pose1}")
            print("The controller accepted '20.0000'-style numbers in WayPoint.")
        return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        sys.exit(f"stopped: {exc}")
