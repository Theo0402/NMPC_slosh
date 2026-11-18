"""The mock controller, driven through the unchanged elfin_min.py (codebase plan §3.8 item 10)."""
import socket
import time

import pytest

from tools.elfin_mock import sim
from tools.elfin_py import datasheet, protocol
from tools.elfin_py.elfin_min import Elfin, ElfinError


def connect(srv):
    return Elfin("127.0.0.1", srv.cmd_port, timeout=2.0)


def test_reads_with_fragmented_replies(mock):
    with connect(mock) as r:
        assert r.cmd("ReadRobotModel") == ["MOCK"]
        st = r.state()
        assert st["nElectrify"] == 1 and st["nEnableState"] == 0
        joints, pose, tool, user = r.act_pos()
        assert joints == sim.MOCK_HOME_DEG and tool == [0.0] * 6 and user == [0.0] * 6
        assert protocol.parse_reply("ReadLinearMaxVel", r.cmd("ReadLinearMaxVel", 0))["dMaxAcc"] == 2500.0


def test_messages_sent_together_are_dropped_after_the_first(mock):
    s = socket.create_connection(("127.0.0.1", mock.cmd_port))
    s.sendall(b"ReadRobotModel,;ReadControllerState,;")
    time.sleep(0.2)
    s.settimeout(0.3)
    data = b""
    try:
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk
    except socket.timeout:
        pass
    s.close()
    assert data == b"ReadRobotModel,OK,MOCK,;"
    assert mock.arm.stats["dropped_extra_messages"] == 1


def test_format_errors_and_forbidden_commands_fail(mock):
    with connect(mock) as r:
        with pytest.raises(ElfinError, match="990001"):
            r.cmd("StartServo", "0", "0.2")                  # wrong parameter count
        with pytest.raises(ElfinError, match="990002"):
            r.cmd("Electrify")
        with pytest.raises(ElfinError, match="990011"):
            r.cmd("SetOverride", "0", "0.5000")
    assert mock.arm.stats["forbidden"] == 1 and mock.arm.stats["format_errors"] == 1


def test_motion_needs_enable_and_joint_move_runs(mock):
    with connect(mock) as r:
        wp = ["WayPoint", *protocol.format_params("WayPoint", 0, *sim.MOCK_HOME_POSE,
                                                  0, 0, 90, 0, 90, 10.0, "TCP", "Base", 20, 100, 0, 0, 1, 0, 0, 0, "1")]
        with pytest.raises(ElfinError, match="990003"):
            r.cmd(*wp)
        r.cmd("GrpEnable", "0")
        r.wait_for(lambda s: s["nEnableState"] == 1, timeout=2)
        t0 = time.monotonic()
        r.cmd(*wp)
        r.wait_done(timeout=5)
        assert 0.6 < time.monotonic() - t0 < 1.5        # 10 deg at 20 deg/s with 100 deg/s^2 ramps = 0.7 s
        assert abs(r.act_pos()[0][5] - 10.0) < 1e-6


def test_servo_stream_tracks_and_grpstop_ends_it(mock):
    with connect(mock) as r:
        r.cmd("GrpEnable", "0")
        r.wait_for(lambda s: s["nEnableState"] == 1, timeout=2)
        r.cmd("StartServo", "0", "0.0200", "0.0200")
        q = list(sim.MOCK_HOME_DEG)
        for k in range(50):
            q[5] = 0.05 * k
            r.cmd("PushServoJ", *protocol.format_params("PushServoJ", 0, *q))
            time.sleep(0.02)
        time.sleep(0.3)        # pushes stop; the setpoint finishes its ramp (the ladder's end-of-stream wait)
        assert abs(r.act_pos()[0][5] - q[5]) < 0.01
        r.cmd("GrpStop", "0")
        r.wait_for(lambda s: s["nMovingState"] == 0, timeout=2)
        assert abs(r.act_pos()[0][5] - q[5]) < 0.01
        with pytest.raises(ElfinError, match="990005"):
            r.cmd("PushServoJ", *protocol.format_params("PushServoJ", 0, *q))


def test_injected_fail_late_reply_and_drop(mock_factory):
    srv = mock_factory(fail_rules=[("ReadRobotModel", 2, 40014)],
                       late_reply_rules=[("ReadControllerState", 1, 0.3)], drop_after_messages=4)
    with connect(srv) as r:
        r.cmd("ReadRobotModel")
        with pytest.raises(ElfinError, match="40014"):
            r.cmd("ReadRobotModel")
        t0 = time.perf_counter()     # monotonic() ticks at ~15 ms on Windows
        r.cmd("ReadControllerState")
        assert time.perf_counter() - t0 >= 0.29
        r.cmd("ReadControllerState")
        with pytest.raises((ConnectionError, OSError)):
            r.cmd("ReadControllerState")


def test_injected_error_stops_streaming(mock_factory):
    srv = mock_factory(error_after_pushes=(3, sim.E_INJECTED_ERROR, 5))
    with connect(srv) as r:
        r.cmd("GrpEnable", "0")
        r.wait_for(lambda s: s["nEnableState"] == 1, timeout=2)
        r.cmd("StartServo", "0", "0.2000", "0.0200")
        q = protocol.format_params("PushServoJ", 0, *sim.MOCK_HOME_DEG)
        for _ in range(3):
            r.cmd("PushServoJ", *q)
        with pytest.raises(ElfinError, match="990008"):
            r.cmd("PushServoJ", *q)
        st = r.state()
        assert st["nErrorState"] == 1 and st["nErrorAxis"] == 5


@pytest.mark.parametrize("order", ["<", ">"])
def test_datasheet_quirks_and_byte_order(mock_factory, order):
    srv = mock_factory(datasheet_byte_order=order)
    rd = datasheet.DatasheetReader("127.0.0.1", srv.state_port)
    try:
        packets = [next(rd) for _ in range(4)]
    finally:
        rd.close()
    arrivals = [p[0] for p in packets]
    assert all(b > a for a, b in zip(arrivals, arrivals[1:]))
    raw, d = packets[-1][1], packets[-1][2]
    assert b'"nAxisGroupErrorCode":[0],}' in raw                     # the manual's trailing comma
    assert isinstance(d["PosAndVel"]["Actual_Position"][0], str)     # string numbers
    assert rd.byte_orders == {order}
    st = datasheet.parse_state(d)
    assert abs(st["q_rad"][2] - 3.14159265 / 2) < 1e-6 and st["state"]["robotEnabled"] == 0
    assert "StateAndError.robotEnabled" in datasheet.key_tree(d)


def test_second_command_client_is_refused(mock):
    with connect(mock) as r:
        r.cmd("ReadRobotModel")
        s2 = socket.create_connection(("127.0.0.1", mock.cmd_port))
        s2.settimeout(1)
        assert s2.recv(10) == b""
        s2.close()
        assert r.cmd("ReadRobotModel") == ["MOCK"]
