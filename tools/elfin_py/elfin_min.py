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
