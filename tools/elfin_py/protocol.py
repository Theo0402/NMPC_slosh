"""The command table: the protocol reference for the HansRobot V5 TCP protocol (codebase plan R-B).

Every entry is a format line as elfin_commands.md gives it, with the PROTO page it cites.
Parameter types come from the names' prefixes (owner's decision, week 1):
    n...           integer, sent as decimal text
    d...           float, sent with 4 decimals ("20.0000"); NaN and infinity are refused
    s..., str...   string, sent as is; commas, semicolons and non-printable ASCII are refused
The C++ client is tested byte-for-byte against `encode()` (codebase plan §9). `elfin_min.py`
stays unchanged: the wrapper formats parameters here and passes strings to `Elfin.cmd()`,
which sends strings as they are.

Whether the controller accepts "20.0000" where the manual's examples show "20" is
[UNTESTED]. If it refuses, add an entry to FORMAT_OVERRIDES; nothing else changes.

`use` says who may send a command:
    used       the tools send it;
    unused     documented, but no tool sends it; the wrapper refuses it;
    forbidden  never sent (owner, week 1): Electrify, StartMaster, ConnectToBox, BlackOut, OSCmd.
Force-sensor commands are left out of the table entirely (owner, week 1: the camera
replaced the F/T sensor; codebase plan §10).

Standard library only (codebase plan §2).
"""
import math
from collections import namedtuple

Param = namedtuple("Param", "name type")
Command = namedtuple("Command", "name params reply page use note")

USED, UNUSED, FORBIDDEN = "used", "unused", "forbidden"

FORBIDDEN_NAMES = ("Electrify", "StartMaster", "ConnectToBox", "BlackOut", "OSCmd")


def _ptype(name):
    if name.startswith("str") or (name[:1] == "s" and name[1:2].isupper()):
        return "str"
    if name[:1] == "n" and name[1:2].isupper():
        return "int"
    if name[:1] == "d" and name[1:2].isupper():
        return "float"
    raise ValueError(f"cannot type parameter {name!r} from its prefix")


def _params(fmt):
    parts = fmt.split(",")
    if parts[-1] != ";":
        raise ValueError(f"format must end with ',;': {fmt!r}")
    return parts[0], tuple(Param(p, _ptype(p)) for p in parts[1:-1])


def _expand(fields):
    out = []
    for f in fields.split():
        out.append(f)
    return tuple(Param(f, _ptype(f)) for f in out)


_J = "dJ1 dJ2 dJ3 dJ4 dJ5 dJ6"
_POSE = "dX dY dZ dRx dRy dRz"
_STATE13 = ("nMovingState nEnableState nErrorState nErrorCode nErrorAxis nBreaking nPause "
            "nEmergencyStop nSaftyGuard nElectrify nIsConnectToBox nBlendingDone nInPos")  # PROTO p.61

_TABLE = [
    # --- connect and power (elfin_commands.md, "Connect and power") ---
    ("ReadControllerState,;", "nStarted", 15, USED, "1 = controller has finished starting"),
    ("ReadRobotModel,;", "strModel", 16, USED, "the manual's example reply is 5KG"),
    ("IsSimulation,;", "nIsSimulation", 14, USED, "0 = real robot, 1 = simulated; reply field name is ours"),
    ("ConnectToBox,;", "", 9, FORBIDDEN,
     "[UNCLEAR] the format line has no parameter but the example sends 0"),
    ("StartMaster,;", "", 12, FORBIDDEN, ""),
    ("CloseMaster,;", "", 13, UNUSED, "disconnects and shuts the master down"),
    ("Electrify,;", "", 10, FORBIDDEN, "power the arm on: done from the pendant instead"),
    ("BlackOut,;", "", 11, FORBIDDEN, ""),
    ("OSCmd,nCmdType,;", "", 8, FORBIDDEN,
     "[UNCLEAR] whether type 1 also shuts the controller down; the example OSCmd,1; lacks a comma"),
    # --- enable, reset, stop (all Name,nRbtID,; replying Name,OK,;) ---
    ("GrpEnable,nRbtID,;", "", 17, USED, "releases the brakes (LINUX p.17)"),
    ("GrpDisable,nRbtID,;", "", 18, USED, ""),
    ("GrpReset,nRbtID,;", "", 19, USED, "only after the operator has seen the error (codebase plan §3.8 item 3)"),
    ("GrpStop,nRbtID,;", "", 20, USED, "effect on servo mode [UNTESTED]"),
    ("GrpInterrupt,nRbtID,;", "", 21, UNUSED, ""),
    ("GrpContinue,nRbtID,;", "", 22, UNUSED, ""),
    # --- settings and limits ---
    ("SetOverride,nRbtID,dOverride,;", "", 46, UNUSED, "0.01-1; cannot change while moving"),
    ("SetPayload,nRbtID,dMass,dX,dY,dZ,;", "", 48, UNUSED,
     "kg, mm; payload is set on the pendant instead (codebase plan §3.8 item 8)"),
    ("SetJointMaxVel,nRbtID,dJ1,dJ2,dJ3,dJ4,dJ5,dJ6,;", "", 49, UNUSED, "deg/s, 0-200"),
    ("SetJointMaxAcc,nRbtID,dJ1,dJ2,dJ3,dJ4,dJ5,dJ6,;", "", 50, UNUSED, "deg/s^2, 0-720"),
    ("SetLinearMaxVel,nRbtID,dMaxVel,;", "", 51, UNUSED, "mm/s, 0-3000"),
    ("SetLinearMaxAcc,nRbtID,dMaxAcc,;", "", 52, UNUSED,
     "mm/s^2, 0-2500; [UNCLEAR] the format names the parameter dMaxVel, the table dMaxAcc"),
    ("ReadJointMaxVel,nRbtID,;", _J, 56, USED, "deg/s"),
    ("ReadJointMaxAcc,nRbtID,;", _J, 57, USED, "deg/s^2"),
    ("ReadJointMaxJerk,nRbtID,;", _J, 58, USED, "deg/s^3, read only"),
    ("ReadLinearMaxVel,nRbtID,;", "dMaxVel dMaxAcc dMaxJerk", 59, USED,
     "mm/s, mm/s^2, mm/s^3; [UNCLEAR] the format line lists dJ1-dJ6, the table and example 3 values"),
    # --- motion ---
    ("WayPoint,nRbtID,dX,dY,dZ,dRx,dRy,dRz,dJ1,dJ2,dJ3,dJ4,dJ5,dJ6,sTcpName,sUcsName,"
     "dVelocity,dAcc,dRadius,nMoveType,nIsUseJoint,nIsSeek,nIOBit,nIOState,strCmdID,;", "", 137, USED,
     "p.137-139; nMoveType 0 joint, 1 linear; reply fields after OK not stated [UNTESTED]; any are accepted and logged"),
    ("MoveRelJ,nRbtID,nAxisId,nDirection,dDistance,;", "", 129, UNUSED,
     "[UNCLEAR] the unit (deg) is only in the example's comment and the format lacks the last comma"),
    ("MoveRelL,nRbtID,nAxisId,nDirection,dDistance,nToolMotion,;", "", 130, UNUSED,
     "[UNCLEAR] the unit (mm) is only in the example's comment; rotation unit [MISSING]"),
    # --- servo streaming (p.172-175) ---
    ("StartServo,nRbtID,dServoTime,dLookaheadTime,;", "", 172, USED, "s, s; both > 0"),
    ("PushServoJ,nRbtID,dJ1,dJ2,dJ3,dJ4,dJ5,dJ6,;", "", 173, USED,
     "deg; the failure example shows code 40014, meaning not given"),
    ("PushServoP,nRbtID,dX,dY,dZ,dRx,dRy,dRz,dUcs_X,dUcs_Y,dUcs_Z,dUcs_Rx,dUcs_Ry,dUcs_Rz,"
     "dTcp_X,dTcp_Y,dTcp_Z,dTcp_Rx,dTcp_Ry,dTcp_Rz,;", "", 174, UNUSED,
     "[UNCLEAR] the parameter table lists the tool frame before the user frame"),
    # --- read state ("all sent as Name,0,;"; the parameter is nRbtID by analogy) ---
    ("ReadRobotState,nRbtID,;", _STATE13, 61, USED, "p.61-62"),
    ("ReadActPos,nRbtID,;",
     _J + " " + _POSE + " dTcp_X dTcp_Y dTcp_Z dTcp_Rx dTcp_Ry dTcp_Rz"
     " dUcs_X dUcs_Y dUcs_Z dUcs_Rx dUcs_Ry dUcs_Rz", 66,
     USED, "p.66-67; joints deg, pose mm/deg, then the current tool frame and user frame"),
    ("ReadCmdPos,nRbtID,;", _J + " " + _POSE, 68, USED,
     "[UNCLEAR] the failure example is labelled ReadActPos (p.69)"),
    ("ReadCmdJointVel,nRbtID,;", _J, 70, USED, "deg/s"),
    ("ReadActJointVel,nRbtID,;", _J, 71, USED, "deg/s"),
    ("ReadCmdTcpVel,nRbtID,;", _POSE, 72, USED, "mm/s, deg/s"),
    ("ReadActTcpVel,nRbtID,;", _POSE, 73, USED, "mm/s, deg/s"),
    ("ReadCmdJointCur,nRbtID,;", _J, 74, USED, "A"),
    ("ReadActJointCur,nRbtID,;", _J, 75, USED, "A"),
    ("ReadEmergencyInfo,nRbtID,;", "nESTO_Error nESTO nSafetyGuard_Error nSafetyGuard", 60, USED,
     "_Error = the two redundant channels disagree"),
    ("ReadAxisErrorCode,nRbtID,;", "nGroupCode nJ1Code nJ2Code nJ3Code nJ4Code nJ5Code nJ6Code", 64, USED,
     "group error code, then J1-J6 codes, 0 = none; field names are ours"),
    ("ReadCurFSM,nRbtID,;", "nFSM", 65, USED, "state-machine table in the missing ErrorCode document"),
]

COMMANDS = {}
for _fmt, _reply, _page, _use, _note in _TABLE:
    _name, _ps = _params(_fmt)
    COMMANDS[_name] = Command(_name, _ps, _expand(_reply), _page, _use, _note)
assert all(COMMANDS[n].use == FORBIDDEN for n in FORBIDDEN_NAMES)

# Per-parameter formatting overrides, keyed by (command, parameter). Empty until the arm
# refuses something. "plain_int" sends an integral float as "20" instead of "20.0000".
FORMAT_OVERRIDES = {}


class ProtocolError(ValueError):
    """A message or reply that does not match the command table."""


class RefusedCommand(ProtocolError):
    """A command the tools may not send (unused or forbidden)."""


def format_param(cmd, param, value):
    override = FORMAT_OVERRIDES.get((cmd, param.name))
    if param.type == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ProtocolError(f"{cmd}.{param.name}: integer required, got {value!r}")
        return str(value)
    if param.type == "float":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError(f"{cmd}.{param.name}: number required, got {value!r}")
        x = float(value)
        if not math.isfinite(x):
            raise ProtocolError(f"{cmd}.{param.name}: {x!r} is not finite; refused")
        if override == "plain_int":
            if x != int(x):
                raise ProtocolError(f"{cmd}.{param.name}: override plain_int needs an integral value, got {x!r}")
            return str(int(x))
        return f"{x:.4f}"
    if not isinstance(value, str) or value == "":
        raise ProtocolError(f"{cmd}.{param.name}: non-empty string required, got {value!r}")
    if any(c in value for c in ",;") or not all(32 < ord(c) < 127 for c in value):
        raise ProtocolError(f"{cmd}.{param.name}: string must be printable ASCII without , ; or spaces: {value!r}")
    return value


def format_params(name, *values):
    """Typed values -> parameter strings, as the wrapper passes them to Elfin.cmd()."""
    c = COMMANDS.get(name)
    if c is None:
        raise ProtocolError(f"{name}: not in the command table")
    if len(values) != len(c.params):
        raise ProtocolError(f"{name}: {len(c.params)} parameters expected, got {len(values)}")
    return [format_param(name, p, v) for p, v in zip(c.params, values)]


def encode(name, *values):
    """The exact message text: Name,p1,...,pn,; (elfin_commands.md, PROTO p.3, p.7)."""
    return ",".join([name, *format_params(name, *values)]) + ",;"


def check_sendable(name):
    c = COMMANDS.get(name)
    if c is None:
        raise RefusedCommand(f"{name}: not in the command table")
    if c.use == FORBIDDEN:
        raise RefusedCommand(f"{name}: forbidden (owner, week 1: never sent by any tool)")
    if c.use != USED:
        raise RefusedCommand(f"{name}: documented but not used by any tool")
    return c


def _parse_field(p, text, where):
    if p.type == "int":
        try:
            # replies show integers; tolerate "1.000"-style text only if it is integral
            v = float(text)
        except ValueError:
            raise ProtocolError(f"{where}.{p.name}: integer expected, got {text!r}") from None
        if v != int(v):
            raise ProtocolError(f"{where}.{p.name}: integer expected, got {text!r}")
        return int(v)
    if p.type == "float":
        try:
            v = float(text)
        except ValueError:
            raise ProtocolError(f"{where}.{p.name}: number expected, got {text!r}") from None
        return v
    return text


def is_action(name):
    """A command whose table entry has no reply fields: it acts (enable, stop, move, push) rather than reads."""
    return not COMMANDS[name].reply


def parse_reply(name, fields):
    """Fields after OK (as Elfin.cmd() returns them) -> dict by the table's reply names.

    An action command's OK is accepted whatever follows it, and {} is returned: OK means the controller
    accepted the command (Name,OK,r1,...,rn,; elfin_commands.md, PROTO p.3; motion commands reply on
    acceptance, PROTO p.1, p.189), so raising afterwards would only skip what the tool does next (its
    stop and exit sequence included). Which fields follow OK is not stated for WayPoint (PROTO p.137-139),
    StartServo or PushServoJ (PROTO p.172-173) [UNTESTED]; the caller logs any it gets. A read reply of
    another shape still raises: its values would land under the wrong names."""
    c = COMMANDS[name]
    if not c.reply:
        return {}
    if len(fields) != len(c.reply):
        raise ProtocolError(f"{name}: {len(c.reply)} reply fields expected, got {len(fields)}: {fields!r}")
    return {p.name: _parse_field(p, f, name) for p, f in zip(c.reply, fields)}


def decode_message(text):
    """Parse one command message as the controller would see it (used by the mock).
    Returns (name, typed params). Accepts integer text for float parameters, as the
    manual's own examples use it."""
    if not text.endswith(",;"):
        raise ProtocolError(f"message must end with ',;': {text!r}")
    parts = text[:-2].split(",")
    name, raw = parts[0], parts[1:]
    if raw == [""]:
        raw = []
    c = COMMANDS.get(name)
    if c is None:
        raise ProtocolError(f"unknown command {name!r}")
    if len(raw) != len(c.params):
        raise ProtocolError(f"{name}: {len(c.params)} parameters expected, got {len(raw)}")
    values = []
    for p, r in zip(c.params, raw):
        if p.type == "int":
            if not r.lstrip("-").isdigit():
                raise ProtocolError(f"{name}.{p.name}: integer text expected, got {r!r}")
            values.append(int(r))
        elif p.type == "float":
            try:
                x = float(r)
            except ValueError:
                raise ProtocolError(f"{name}.{p.name}: number text expected, got {r!r}") from None
            if not math.isfinite(x) or any(ch in r.lower() for ch in "einf"):
                raise ProtocolError(f"{name}.{p.name}: fixed-point text expected, got {r!r}")
            values.append(x)
        else:
            if r == "":
                raise ProtocolError(f"{name}.{p.name}: empty string")
            values.append(r)
    return name, values
