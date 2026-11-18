"""Load and check config/arm/confirmed.yaml and config/arm/e03pro.yaml (standard library only)."""
import pathlib

from tools import yamlmini

ROOT = pathlib.Path(__file__).resolve().parents[2]
CONFIRMED = ROOT / "config" / "arm" / "confirmed.yaml"
ARM = ROOT / "config" / "arm" / "e03pro.yaml"

CONFIRMED_KEYS = ("joint_speed_deg_s", "joint_acc_deg_s2", "linear_speed_mm_s", "linear_acc_mm_s2",
                  "stream_joint_speed_deg_s", "stream_joint_acc_deg_s2", "servo_period_s", "servo_lookahead_s")


class ConfigError(ValueError):
    pass


def _positive(d, keys, where):
    for k in keys:
        v = d.get(k)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not v > 0:
            raise ConfigError(f"{where}: {k} must be a positive number, got {v!r}")


def load(confirmed_path=CONFIRMED, arm_path=ARM):
    """Return (confirmed, arm, texts). texts are the files verbatim, for the log header."""
    texts = {"confirmed.yaml": pathlib.Path(confirmed_path).read_text(encoding="utf-8"),
             "e03pro.yaml": pathlib.Path(arm_path).read_text(encoding="utf-8")}
    confirmed = yamlmini.loads(texts["confirmed.yaml"], "confirmed.yaml")
    arm = yamlmini.loads(texts["e03pro.yaml"], "e03pro.yaml")
    missing = [k for k in CONFIRMED_KEYS if k not in confirmed]
    if missing:
        raise ConfigError(f"confirmed.yaml lacks {missing}")
    _positive(confirmed, CONFIRMED_KEYS, "confirmed.yaml")
    servo = arm.get("servo_in_use") or {}
    if servo.get("period_s", 0) < confirmed["servo_period_s"]:
        raise ConfigError("e03pro.yaml servo_in_use.period_s is shorter than the confirmed servo period")
    for k in ("tool_frame", "user_frame"):
        if len(arm["mounting"][k]) != 6:
            raise ConfigError(f"e03pro.yaml mounting.{k} needs 6 values")
    return confirmed, arm, texts


def limits_known(arm):
    lr = arm.get("limits_readback") or {}
    return all(lr.get(k) is not None for k in
               ("record_id", "joint_max_vel_deg_s", "joint_max_acc_deg_s2", "joint_max_jerk_deg_s3", "linear_max"))
