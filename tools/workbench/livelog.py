"""Read a trial log while it is being written (codebase plan §5: append-only records, tag + length +
payload; a truncated last record is the end so far). Used for the workbench's live plots only."""
import json
import pathlib

from analysis.logread import latest

L = latest()
RAD = 57.29577951308232


class LiveLog:
    def __init__(self, path, max_points=1500):
        self.path = pathlib.Path(path)
        self.pos = 0
        self.header = None
        self.max_points = max_points
        self.cmd = []        # (t_s, q_deg x6)
        self.state = []      # (t_s, q_deg x6)
        self.events = []     # (t_s, kind, text)
        self.t0 = None

    def _t(self, ns):
        if self.t0 is None:
            self.t0 = ns
        return (ns - self.t0) / 1e9

    def poll(self):
        if not self.path.exists():
            return
        with open(self.path, "rb") as f:
            f.seek(self.pos)
            blob = f.read()
        i = 0
        if self.header is None:
            magic = L.MAGIC["bin"]
            if len(blob) < len(magic) + 4:
                return
            (n,) = L._HDR.unpack_from(blob, len(magic))
            start = len(magic) + L._HDR.size
            if len(blob) < start + n:
                return
            self.header = json.loads(blob[start:start + n].decode("utf-8"))
            self.t0 = self.header.get("monotonic_ns_at_header")
            i = start + n
        while i + L._REC.size <= len(blob):
            tag, length = L._REC.unpack_from(blob, i)
            if i + L._REC.size + length > len(blob):
                break                                    # the rest has not been written yet
            s = L.TAGS.get(tag)
            if s is not None:
                rec = L.unpack(s.name, blob[i + L._REC.size:i + L._REC.size + length])
                self._take(s.name, rec)
            i += L._REC.size + length
        self.pos += i

    def _take(self, name, r):
        if name == "arm_cmd":
            self.cmd.append((self._t(r["t_send_ns"]), [x * RAD for x in r["q_cmd_rad"]]))
        elif name == "robot_state" and r["valid"] & 1:
            self.state.append((self._t(r["t_host_ns"]), [x * RAD for x in r["q_rad"]]))
        elif name == "event":
            self.events.append((self._t(r["t_host_ns"]), r["kind"], r["text"].decode("utf-8", "replace")[:200]))

    @staticmethod
    def _thin(series, n):
        if len(series) <= n:
            return series
        step = len(series) / n
        return [series[int(k * step)] for k in range(n)] + [series[-1]]

    def snapshot(self):
        return {"cmd": self._thin(self.cmd, self.max_points), "state": self._thin(self.state, self.max_points),
                "events": self.events[-30:], "n_cmd": len(self.cmd), "n_state": len(self.state)}
