"""Bring-up read (example 1, extended). Nothing moves and nothing is enabled.

    python -m tools.elfin_py.bringup_read --dry-run | --mock | --arm [--controller-version TEXT]

Reads, each through the command table (elfin_commands.md): ReadControllerState (PROTO p.15),
ReadRobotModel (p.16), IsSimulation (p.14), ReadRobotState (p.61), ReadEmergencyInfo (p.60),
ReadAxisErrorCode (p.64), ReadCurFSM (p.65), ReadActPos (p.66), ReadJointMaxVel/Acc/Jerk
(p.56-58) and ReadLinearMaxVel (p.59). Prints the limits as a block to paste into
config/arm/e03pro.yaml (limits_readback), with this trial's ID as the record ID, and compares
them with what is there. A reply whose shape differs from the table is reported, not fatal.
"""
import sys

from tools.elfin_py import cli, link, protocol, triallog
from tools.elfin_py.elfin_min import ElfinError

FIRST_RUN = ("The model string and the limits read back; the current tool and user frames are zero\n"
             "(bare flange, owner week 1). Nothing moves; the arm is not enabled.")
READS = ("ReadControllerState", "ReadRobotModel", "IsSimulation", "ReadRobotState", "ReadEmergencyInfo",
         "ReadAxisErrorCode", "ReadCurFSM", "ReadActPos", "ReadJointMaxVel", "ReadJointMaxAcc",
         "ReadJointMaxJerk", "ReadLinearMaxVel")


def main(argv=None):
    ap = cli.parser(__doc__)
    ap.add_argument("--controller-version", default=None, help="as the pendant's About screen shows it (LINUX p.19)")
    args = ap.parse_args(argv)
    with cli.session(args, "bringup_read", "scratch", FIRST_RUN) as (arm, log, confirmed, arm_cfg, ask):
        res, problems = {}, []
        for name in READS:
            values = [0] * len(protocol.COMMANDS[name].params)
            try:
                res[name] = arm.cmd(name, *values)
            except (ElfinError, protocol.ProtocolError) as exc:
                res[name] = None
                problems.append(f"{name}: {exc}")
            print(f"{name:20s} {res[name]}")
        block = {"trial_id": log.trial_id, "controller_version_entered": args.controller_version,
                 "reads": res, "problems": problems}
        log.event(triallog.EV_ARM_BLOCK, block)

        pos = res.get("ReadActPos")
        if pos:
            v = list(pos.values())
            tool, user = v[12:18], v[18:24]
            problems += link.frame_problems(tool, user, arm_cfg["mounting"])
        if all(res.get(n) for n in ("ReadJointMaxVel", "ReadJointMaxAcc", "ReadJointMaxJerk", "ReadLinearMaxVel")):
            lim = {"joint_max_vel_deg_s": list(res["ReadJointMaxVel"].values()),
                   "joint_max_acc_deg_s2": list(res["ReadJointMaxAcc"].values()),
                   "joint_max_jerk_deg_s3": list(res["ReadJointMaxJerk"].values()),
                   "linear_max": list(res["ReadLinearMaxVel"].values())}
            lr = arm_cfg.get("limits_readback") or {}
            for k, v in lim.items():
                if lr.get(k) is not None and lr[k] != v:
                    problems.append(f"{k}: read {v}, e03pro.yaml has {lr[k]}")
            print("\nPaste into config/arm/e03pro.yaml (limits_readback), if these are the arm's values:")
            print(f"limits_readback:\n  record_id: {log.trial_id}")
            for k, v in lim.items():
                print(f"  {k}: [{', '.join(f'{x:.3f}' for x in v)}]")
            print("  tolerance: 0.001")
        model = (res.get("ReadRobotModel") or {}).get("strModel")
        print(f"\nmodel string: {model!r} (the manual's example reply is '5KG'; elfin_commands.md, PROTO p.16)")
        if problems:
            print("\nPROBLEMS:\n  " + "\n  ".join(problems))
            log.event(triallog.EV_ERROR, {"problems": problems})
        return 1 if problems else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:   # the session has logged and closed the trial
        sys.exit(f"stopped: {exc}")
