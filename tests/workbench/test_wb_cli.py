"""The command-line client (tools/wb.py, owner request 2026-11-25): the workbench without a browser. It must go through
the workbench's API only, so every check stays the workbench's, and it must never answer an arm question by itself."""
import threading

import pytest

from tests.workbench.test_arm_control import bench  # noqa: F401 (a fixture)
from tools import wb
from tools.workbench.server import serve


@pytest.fixture
def cli(bench, monkeypatch, capsys):  # noqa: F811
    httpd = serve(bench, port=0)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    monkeypatch.setattr(wb, "PORT", httpd.server_address[1])
    monkeypatch.setattr(wb, "data_dir", lambda: bench.data_dir)

    def run(*args):
        code = wb.main(list(args))
        return code, capsys.readouterr().out
    yield run
    if bench.run is not None and bench.run.running:
        bench.run.stop()
        bench.run.thread.join(timeout=30)
    httpd.shutdown()
    httpd.server_close()


def test_questions_wait_for_an_answer_then_jog_goto_and_end(cli):
    code, out = cli("run", "arm_control", "mock", "--note", "cli test")
    assert code == 0 and "QUESTION 1:" in out and "READY" not in out          # nothing answered by itself
    out = cli("answer", "1", "yes")[1]
    assert "QUESTION 2:" in out
    out = cli("answer", "2", "yes")[1]
    assert "READY: the arm is enabled." in out and "X 420.0" in out
    assert "done" in cli("jog", "x", "10")[1] and "X 430.0" in cli("pose")[1]
    assert "REFUSED: step 500.0 mm" in cli("jog", "z", "-500")[1]
    assert "X 425.0  Y 5.0  Z 440.0" in cli("goto", "425", "5", "440")[1]
    out = cli("end")[1]
    assert "ENDED: exit code 0" in out


def test_corners_become_the_workspace_box(cli, bench):
    cli("run", "arm_control", "mock", "--yes")
    cli("corner")
    cli("jog", "x", "20"), cli("jog", "y", "20"), cli("jog", "z", "-20")
    assert "earlier corner(s) kept" in cli("corner")[1]
    out = cli("ws-save", "--note", "cli test", "--margin", "2")[1]
    assert "[422.0, 438.0, 2.0, 18.0, 427.0, 443.0]" in out and not wb.load_corners()
    assert bench.workspace_view()["box"] == [422.0, 438.0, 2.0, 18.0, 427.0, 443.0]
    assert "ENDED: exit code 2" in cli("stop")[1]                                  # STOP waits for the end


def test_an_analysis_stage_runs_from_the_cli(cli):
    code, out = cli("run", "host_check", "analysis", "--note", "cli test")
    assert code == 0 and "started" in out and "ENDED: exit code" in out


def test_yes_is_refused_on_the_arm_and_errors_are_plain(cli):
    code, _ = cli("run", "move_test", "arm", "--yes")
    assert code == 1
    assert cli("jog", "x", "5")[0] == 1                                            # no session: a plain refusal
