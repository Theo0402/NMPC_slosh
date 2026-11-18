"""Datasheet reader for port 10004, adapted from example 4 (elfin_examples.md).

Packet (elfin_commands.md, PROTO p.188): "RBTL" (4 bytes), total length including the header
(4 bytes), data length (4 bytes), then JSON. Byte order and signedness are [MISSING]; the
order is the one for which total = data + 12. The manual's JSON example has numbers as
strings and a trailing comma after "nAxisGroupErrorCode":[0] (PROTO p.185-186); whether the
real controller does the same is [UNTESTED]. There is no timestamp, so each packet is
stamped on arrival with time.monotonic_ns() (codebase plan §5).

Standard library only.
"""
import json
import math
import re
import socket
import struct
import time

_TRAILING_COMMA = re.compile(rb",\s*([}\]])")


class DatasheetError(ValueError):
    pass


def split_header(head):
    """12 header bytes -> (total, data_length, byte_order)."""
    if len(head) != 12:
        raise DatasheetError(f"short header: {head.hex()}")
    if head[:4] != b"RBTL":
        raise DatasheetError(f"no RBTL marker: {head.hex()}")
    for order in "<>":
        total, n = struct.unpack(order + "II", head[4:])
        if total == n + 12:
            return total, n, order
    raise DatasheetError(f"lengths fit neither byte order: {head.hex()}")


def decode_body(body):
    """JSON body bytes -> dict, tolerating the manual's trailing comma."""
    return json.loads(_TRAILING_COMMA.sub(rb"\1", body).decode("utf-8"))


def key_tree(d, prefix=""):
    """Sorted list of every key path in a packet, e.g. 'StateAndError.robotEnabled'."""
    out = []
    for k, v in d.items():
        path = f"{prefix}{k}"
        out.append(path)
        if isinstance(v, dict):
            out.extend(key_tree(v, path + "."))
    return sorted(out)


def _num(x):
    return float(x)   # numbers may be strings ("0.000") in the manual's example


def _vec(d, *keys, n=6):
    for k in keys:
        d = d.get(k) if isinstance(d, dict) else None
    if not isinstance(d, list) or len(d) < n:
        return None
    try:
        return [_num(v) for v in d[:n]]
    except (TypeError, ValueError):
        return None


def _int(d, *keys):
    for k in keys:
        d = d.get(k) if isinstance(d, dict) else None
    try:
        return int(_num(d))
    except (TypeError, ValueError):
        return None


def parse_state(packet):
    """Packet dict -> the robot_state fields we log, in SI (codebase plan §3.3 F4).
    Any group the packet lacks comes back as None, with its valid flag cleared (RT5)."""
    deg = math.pi / 180.0
    pos = _vec(packet, "PosAndVel", "Actual_Position", n=6)
    vel = _vec(packet, "PosAndVel", "Actual_Joint_Velocity")
    acc = _vec(packet, "PosAndVel", "Actual_Joint_Acceleration")
    cur = _vec(packet, "PosAndVel", "Actual_Joint_Current")
    st = {k: _int(packet, "StateAndError", k) for k in
          ("robotState", "robotEnabled", "robotPaused", "robotMoving", "robotBlendingDone", "InPos",
           "Error_AxisID", "Error_Code")}
    return {
        "q_rad": [v * deg for v in pos] if pos else None,
        "qd_rad_s": [v * deg for v in vel] if vel else None,
        "qdd_rad_s2": [v * deg for v in acc] if acc else None,
        "current_A": cur,
        "state": st,
    }


class DatasheetReader:
    """Yields (arrival_ns, raw_packet_bytes, dict) from port 10004."""

    def __init__(self, ip, port=10004, timeout=5.0, on_raw=None):
        self.sock = socket.create_connection((ip, port), timeout=timeout)
        self.stream = self.sock.makefile("rb")
        self.on_raw = on_raw
        self.byte_orders = set()

    def close(self):
        try:
            self.stream.close()
        finally:
            self.sock.close()

    def __iter__(self):
        return self

    def __next__(self):
        head = self.stream.read(12)
        if len(head) < 12:
            raise StopIteration
        _, n, order = split_header(head)
        self.byte_orders.add(order)
        body = self.stream.read(n)
        arrival = time.monotonic_ns()
        if len(body) < n:
            raise StopIteration
        raw = head + body
        if self.on_raw:
            self.on_raw(arrival, raw)
        return arrival, raw, decode_body(body)
