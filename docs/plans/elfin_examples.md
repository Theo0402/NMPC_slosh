# Elfin E03-Pro: Python examples

Short examples for connecting, moving, streaming joint positions and reading the state stream over the HansRobot V5 TCP protocol. Built 22 Oct 2026 from the Elfin control attachments only. Companion files: `elfin_overview.md`, `elfin_commands.md`.

**Correction, 12 Nov 2026.** The arm is an **Elfin E03-Pro**, not an E05-Pro as these files first said. It is the lab's arm, serial MH217340001, the one in the factory inspection report [QC p.1], and the owner reports that the manuals are for the 3 Pro. Facts marked [E03] therefore describe this arm. The model name is corrected throughout.

**Markers.** **[E03]** from the lab's earlier records of this arm (inspection report, MATLAB scripts, ROS config), where it is called an E03; likely to hold, but confirm, since settings and firmware may have changed. **[UNCLEAR]** the sources are ambiguous or contradict each other. **[MISSING]** the attachments don't say. **[UNTESTED]** not tried on hardware.

**Sources.** PROTO = `说明书/HansRobotV5控制通信协议接口.pdf`; LINUX = `说明书/Linux system manual for Elfin cobot.pdf`; NET = `说明书/HansRobot_网络配置.pdf`; MATLAB = `Code/MATLAB/`. Pages are printed footer numbers (PDF page = printed + 7 for PROTO, + 1 for LINUX, + 2 for NET).

**Status: all code is [UNTESTED] on hardware.** The protocol refers to official SDK manuals [PROTO p.192], but none is attached [MISSING], so this code is written from the protocol. On 22 Oct 2026 every block ran against a local mock controller. The mock checked message format and parameter counts, split replies into fragments, and streamed Datasheet packets in the manual's format. That tests framing and parsing only, not the arm's behaviour.

**Before running on the arm:**

- Keep the e-stop in reach and the workspace clear; enabling makes the arm move slightly [LINUX p.17–18].
- Set the environment variable `ELFIN_IP`. The default, `192.168.0.10`, is this arm's address in the lab's scripts [MATLAB Connect_robotarm.m:3]. [E03]
- Close the web client: only one client can use the console at a time [NET p.1]. Whether that includes port 10003: [UNCLEAR]
- Python 3.8 or later, standard library only. Save the first block as `elfin_min.py` next to the examples.

## elfin_min.py: command client

```python
# elfin_min.py: minimal client for the HansRobot V5 TCP protocol (port 10003). [UNTESTED]
import os
import socket
import time

ROBOT_IP = os.environ.get("ELFIN_IP", "192.168.0.10")  # address in the lab's scripts [MATLAB Connect_robotarm.m:3] [E03]

STATE_FIELDS = ("nMovingState nEnableState nErrorState nErrorCode nErrorAxis nBreaking nPause "
                "nEmergencyStop nSaftyGuard nElectrify nIsConnectToBox nBlendingDone nInPos").split()  # PROTO p.61


class ElfinError(RuntimeError):
    pass


def _fmt(p):
    return f"{p:.4f}" if isinstance(p, float) else str(p)  # fixed-point text, never "1e-05"


class Elfin:
    def __init__(self, ip=ROBOT_IP, port=10003, timeout=2.0):
        self.sock = socket.create_connection((ip, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # send short messages at once
        self.buf = b""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.sock.close()

    def cmd(self, name, *params):
        """Send one command, wait for its reply, return the fields after OK.
        Format and replies [PROTO p.3]; one command in flight at a time [PROTO p.1]."""
        self.sock.sendall((",".join([name, *map(_fmt, params)]) + ",;").encode("ascii"))
        while b";" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("controller closed the connection")
            self.buf += chunk
        reply, _, self.buf = self.buf.partition(b";")
        f = [x.strip() for x in reply.decode("ascii", "replace").split(",")]
        if f[-1] == "":
            f.pop()
        if len(f) < 2 or f[0] != name:
            raise ElfinError(f"unexpected reply to {name}: {reply!r}")
        if f[1] != "OK":
            raise ElfinError(f"{name} failed, error code {f[2:]}")  # code list not attached [MISSING]
        return f[2:]

    def state(self):
        return dict(zip(STATE_FIELDS, (int(float(x)) for x in self.cmd("ReadRobotState", 0))))

    def act_pos(self):
        v = [float(x) for x in self.cmd("ReadActPos", 0)]
        return v[0:6], v[6:12], v[12:18], v[18:24]  # joints (deg), pose (mm, deg), tool frame, user frame [PROTO p.66]

    def wait_for(self, test, timeout, period=0.01):
        t_end = time.monotonic() + timeout
        while True:
            s = self.state()
            if test(s):
                return s
            if time.monotonic() > t_end:
                raise TimeoutError(f"gave up waiting; last state {s}")
            time.sleep(period)

    def enable(self, timeout=30.0):
        """Power on if needed, clear errors, enable. Enabling releases the brakes [LINUX p.17]."""
        s = self.state()
        if s["nEmergencyStop"] or s["nSaftyGuard"]:
            raise ElfinError("e-stop or light curtain is active")
        if not s["nElectrify"]:
            self.cmd("Electrify")  # [PROTO p.10]; the lab's scripts never sent it [UNTESTED]
            self.wait_for(lambda st: st["nElectrify"] == 1, timeout)
        if s["nErrorState"]:
            self.cmd("GrpReset", 0)  # [PROTO p.19]
        self.cmd("GrpEnable", 0)  # [PROTO p.17]
        return self.wait_for(lambda st: st["nEnableState"] == 1, timeout)

    def wait_done(self, timeout=60.0, start_grace=0.5):
        """Poll ReadRobotState every 10 ms until nMovingState is 0 [PROTO p.189-191].
        Waits up to start_grace s for the move to start first; that step is not in the manual [MISSING]."""
        t0, started = time.monotonic(), False
        while True:
            s = self.state()
            if s["nErrorState"]:
                raise ElfinError(f"motion error {s['nErrorCode']} on axis {s['nErrorAxis']}; send GrpReset")
            started = started or s["nMovingState"] == 1
            t = time.monotonic() - t0
            if s["nMovingState"] == 0 and (started or t > start_grace):
                return s  # the lab's scripts test nBlendingDone == 1 instead [E03]
            if t > timeout:
                raise TimeoutError("move did not finish")
            time.sleep(0.01)
```

## Example 1: connect and read

Nothing moves.

```python
# example 1: connect and read
from elfin_min import Elfin

with Elfin() as r:
    print("controller started:", r.cmd("ReadControllerState"))  # ['1'] = started [PROTO p.15]
    print("model:", r.cmd("ReadRobotModel"))  # manual's example reply: 5KG [PROTO p.16]
    print("state:", r.state())  # [PROTO p.61]
    joints, pose, tool, user = r.act_pos()  # [PROTO p.66]
    print("joints (deg):", joints)
    print("pose (mm, deg):", pose, "tool frame:", tool, "user frame:", user)
    print("max joint vel (deg/s):", r.cmd("ReadJointMaxVel", 0))  # [PROTO p.56]
    print("max joint acc (deg/s^2):", r.cmd("ReadJointMaxAcc", 0))  # [PROTO p.57]
    print("linear vel, acc, jerk:", r.cmd("ReadLinearMaxVel", 0))  # mm/s, mm/s^2, mm/s^3 [PROTO p.59]
```

## Example 2: enable and move

Turns J6 by +10°, then moves the tool 20 mm up. Speeds are slow values chosen for a first test, inside the ranges on PROTO p.49–52.

```python
# example 2: enable and move
from elfin_min import Elfin

with Elfin() as r:
    r.enable()
    joints, pose, tool, user = r.act_pos()

    # Joint move to a joint target: nMoveType 0, nIsUseJoint 1 [PROTO p.137-139]; 20 deg/s, 100 deg/s^2
    target = joints[:5] + [joints[5] + 10.0]
    r.cmd("WayPoint", 0, *pose, *target, "TCP", "Base", 20, 100, 0, 0, 1, 0, 0, 0, "1")
    r.wait_done()

    # Linear move: nMoveType 1; 20 mm/s, 100 mm/s^2
    joints, pose, tool, user = r.act_pos()
    if any(tool) or any(user):  # pose frames: ReadActPos doesn't say [MISSING]; Datasheet uses the current ones [PROTO p.186]
        raise SystemExit("current tool or user frame is not zero: pass their names instead of TCP/Base")
    target = pose[:2] + [pose[2] + 20.0] + pose[3:]
    r.cmd("WayPoint", 0, *target, *joints, "TCP", "Base", 20, 100, 0, 1, 0, 0, 0, 0, "2")
    r.wait_done()
    print("joints, pose now:", r.act_pos()[:2])
    r.cmd("GrpDisable", 0)  # disabled = brakes on [LINUX p.18]; the lab's script also ends so [MATLAB Connect_robotarm.m:28]
```

## Example 3: stream joint positions

Moves J6 through a slow ±5° sine for 20 s with `StartServo` and `PushServoJ` [PROTO p.172–173]. It uses the manual's example timing, a 0.2 s period and 0.02 s lookahead; no minimum period is documented. [MISSING] It reports how long each push took to be answered and how many were sent late.

```python
# example 3: stream joint positions (servo mode)
import math
import time
from elfin_min import Elfin

SERVO_T = 0.2  # dServoTime, s: the manual's example value [PROTO p.172]
LOOKAHEAD_T = 0.02  # dLookaheadTime, s: the manual's example value [PROTO p.172]
AMP, PERIOD, DURATION = 5.0, 10.0, 20.0  # J6 sine: deg, s, s (slow values for a first test)

with Elfin() as r:
    r.enable()
    q0 = r.act_pos()[0]
    r.cmd("StartServo", 0, SERVO_T, LOOKAHEAD_T)
    replies, late = [], 0
    t0 = time.perf_counter() + SERVO_T  # first push one period from now
    for k in range(int(DURATION / SERVO_T) + 1):
        t = k * SERVO_T
        q = list(q0)
        q[5] += AMP * math.sin(2 * math.pi * t / PERIOD)  # starts and ends at q0
        wait = t0 + t - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        else:
            late += 1
        sent = time.perf_counter()
        r.cmd("PushServoJ", 0, *q)  # degrees [PROTO p.173]
        replies.append(time.perf_counter() - sent)
    # Leaving servo mode is not documented [MISSING]; GrpStop ("stop motion", PROTO p.20) is [UNTESTED]
    r.cmd("GrpStop", 0)
    r.wait_done(timeout=10)
    replies.sort()
    print(f"{len(replies)} pushes, {late} late; reply time median {1e3 * replies[len(replies) // 2]:.1f} ms,"
          f" max {1e3 * replies[-1]:.1f} ms")
    print("J6 start and end (deg):", q0[5], r.act_pos()[0][5])
    r.cmd("GrpDisable", 0)
```

To go faster, shorten `SERVO_T` in steps and watch the reply times and the late count.

## Example 4: read the state stream

Reads the JSON Datasheet on port 10004 for 10 s, prints a line about once a second, and reports the packet spacing (nominally 50 ms at 20 Hz [PROTO p.7]). Packet layout is from PROTO p.188.

```python
# example 4: read the state stream (Datasheet, port 10004)
import json
import os
import re
import socket
import struct
import time

ROBOT_IP = os.environ.get("ELFIN_IP", "192.168.0.10")


def datasheet(ip=ROBOT_IP, port=10004):
    """Yield (arrival time, dict) for each packet [PROTO p.7, p.185-188]. [UNTESTED]"""
    stream = socket.create_connection((ip, port), timeout=5).makefile("rb")
    while True:
        head = stream.read(12)
        if len(head) < 12:
            raise ConnectionError("stream closed")
        if head[:4] != b"RBTL":
            raise ValueError(f"no RBTL marker: {head.hex()}")
        for order in "<>":  # byte order not documented [MISSING]; total = header + data [PROTO p.188]
            total, n = struct.unpack(order + "II", head[4:])
            if total == n + 12:
                break
        else:
            raise ValueError(f"lengths fit neither byte order: {head.hex()}")
        body = stream.read(n)
        arrival = time.time()  # the packet has no timestamp [MISSING]
        text = re.sub(r",\s*([}\]])", r"\1", body.decode("utf-8"))  # manual's example has a trailing comma
        yield arrival, json.loads(text)


arrivals = []
for arrival, d in datasheet():
    arrivals.append(arrival)
    if len(arrivals) % 20 == 1:  # about once a second at 20 Hz
        pv, se = d["PosAndVel"], d["StateAndError"]
        joints = [float(x) for x in pv["Actual_Position"][:6]]  # strings in the manual's example
        print(f"J (deg) {joints}  moving {se['robotMoving']}  enabled {se['robotEnabled']}"
              f"  F/T {d['FTData']['FTData']}")
    if arrival - arrivals[0] > 10:
        break
gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
print(f"{len(arrivals)} packets in {arrivals[-1] - arrivals[0]:.1f} s;"
      f" gap mean {1e3 * sum(gaps) / len(gaps):.1f} ms, max {1e3 * max(gaps):.1f} ms")
```

## Example 5: is a force sensor fitted?

Keep the arm disabled for this check.

```python
# example 5: force sensor check
import time
from elfin_min import Elfin, ElfinError

with Elfin() as r:
    def read():
        return [float(x) for x in r.cmd("ReadFTCabData", 0)]  # N and N*m [PROTO p.128]

    try:
        print("calibrated F/T:", read())
    except ElfinError as e:
        raise SystemExit(f"no force data: {e}")  # what an arm without a sensor replies is [MISSING]
    r.cmd("SetForceZero", 0)  # re-zero [PROTO p.114]
    time.sleep(0.5)  # how long re-zeroing takes is not stated [MISSING]
    zero = read()
    print("after SetForceZero:", zero)
    input("Hang a known mass on the tool flange, then press Enter")
    loaded = read()
    change = sum((a - b) ** 2 for a, b in zip(loaded[:3], zero[:3])) ** 0.5
    print(f"loaded: {loaded}; force change {change:.2f} N (expect about mass x 9.81 N)")
```

The same data streams as `FTData.FTData` and `FTData.FTSrcData` on port 10004 [PROTO p.186, p.188].

## What this code assumes

| Assumption | Basis |
|---|---|
| A reply ends at its first `;` | Every documented reply ends with `,;` [PROTO p.3] |
| Numbers sent with 4 decimals are accepted | The manual's examples show at most 3, in replies [PROTO p.67] and in commands [PROTO p.133] [UNTESTED] |
| `Electrify` is sent only if the arm reports it is unpowered | The lab's scripts never sent it [MATLAB Connect_robotarm.m:5] [UNTESTED] |
| `wait_done` first waits up to 0.5 s for the move to start | Not in the manual [MISSING] |
| `GrpStop` ends servo mode | No exit command is documented [MISSING] [UNTESTED] |
| Datasheet byte order is chosen by total = data + 12 | The manual gives no byte order [PROTO p.188] [MISSING] |

## Suggested first session on the E03-Pro

1. Example 1: check the model string, the speed limits and the current frames.
2. Example 4 on its own: check the packet rate and the JSON format.
3. Example 2, with a hand on the e-stop.
4. Example 3 at the manual's 0.2 s period, then shorter periods. To measure tracking delay, run Example 4's reader in a second thread during Example 3 and compare commanded and measured J6.
5. Example 5, if force data matters.
