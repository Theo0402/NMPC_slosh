"""Record ReadActPos at the joint configurations in config/arm/fk_poses.yaml, for the week-2 FK check.

    python -m tools.elfin_py.pose_recorder --dry-run | --mock | --arm [--poses FILE]

Bare flange, zero TCP and user frame (owner, week 1). Each pose is a joint WayPoint (nMoveType 0,
nIsUseJoint 1; elfin_commands.md, PROTO p.137-139) at the confirmed joint speed and acceleration,
checked against joint_ranges_deg in e03pro.yaml, which must be entered from the pendant first
(the protocol has no read command for them; elfin_overview.md, LINUX p.50). On the arm the
operator confirms each move. At each pose the arm settles for 0.5 s, then ReadActPos is read
three times (PROTO p.66-67). The poses are compared with FK in week 2: positions first,
orientations once the Rx, Ry, Rz convention is established.

The question for pose 1 comes before GrpEnable. The later ones are asked with the arm enabled, so no
answer within QUESTION_TIMEOUT_S is taken as no: the run ends and GrpDisable is sent (pre-hardware
review S9). In a Windows terminal the timeout does not apply (arm runs are Linux only).
"""
import sys
import time

from tools import yamlmini
from tools.elfin_py import cli, config, link, motion, triallog

FIRST_RUN = ("Each pose is a joint move at the confirmed speed (20 deg/s); confirm each one. Watch for anything\n"
             "near the arm's path: the moves go straight between joint configurations, not around obstacles.")
QUESTION_TIMEOUT_S = 300.0     # as jog_idle_timeout_s's default [proposal; owner's question 7 of the review]


def main(argv=None):
    ap = cli.parser(__doc__)
    ap.add_argument("--poses", default=str(config.ROOT / "config" / "arm" / "fk_poses.yaml"))
    args = ap.parse_args(argv)
    poses = (yamlmini.load(args.poses) or {}).get("poses") or []
    with cli.session(args, "pose_recorder", "scratch", FIRST_RUN) as (arm, log, confirmed, arm_cfg, ask):
        if not poses:
            raise link.Refused(f"no poses in {args.poses}")
        ranges = arm_cfg.get("joint_ranges_deg")
        if ranges is None:
            raise link.Refused("joint_ranges_deg is not entered in e03pro.yaml (from the pendant, LINUX p.50)")
        for n, p in enumerate(poses, 1):
            if len(p) != 6 or any(not lo <= v <= hi for v, (lo, hi) in zip(p, ranges)):
                raise link.Refused(f"pose {n} {p} is outside joint_ranges_deg {ranges}")
        v, a = confirmed["joint_speed_deg_s"], confirmed["joint_acc_deg_s2"]
        motion.check_waypoint("joint", v, a, confirmed)
        m = arm_cfg["mounting"]
        records = []

        def question(n, target):
            return f"Move to pose {n}/{len(poses)}: {target} deg?"

        def first_pose():
            if not ask(question(1, poses[0])):
                raise link.Refused("operator stopped before pose 1")

        with arm.disable_on_exit():
            arm.prepare_and_enable(arm_cfg, ask, before_enable=first_pose)
            for n, target in enumerate(poses, 1):
                if n > 1 and not ask(question(n, target) + f" (The arm is enabled: no answer within "
                                     f"{QUESTION_TIMEOUT_S / 60:g} min is taken as no and ends the run.)",
                                     timeout=QUESTION_TIMEOUT_S):
                    raise link.Refused(f"operator stopped before pose {n}")
                arm.check_before_motion()
                _, pose, _, _ = arm.act_pos()
                log.event(triallog.EV_PHASE, f"pose {n}: {target}")
                arm.cmd("WayPoint", 0, *pose, *[float(x) for x in target], m["tcp_name"], m["ucs_name"],
                        float(v), float(a), 0.0, 0, 1, 0, 0, 0, f"P{n}")
                arm.wait_done(joints=[float(x) for x in target])   # never a pose read mid-move (review S12)
                time.sleep(0.5)
                reads = [arm.act_pos() for _ in range(3)]
                rec = {"pose": n, "target_deg": target, "reads": [{"joints": r[0], "pose": r[1], "tool": r[2],
                                                                    "user": r[3]} for r in reads]}
                records.append(rec)
                log.event(triallog.EV_INFO, rec)
                print(f"pose {n}: joints {reads[-1][0]}\n        pose   {reads[-1][1]}")
        log.event(triallog.EV_SUMMARY, {"poses": records})
        return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        sys.exit(f"stopped: {exc}")
