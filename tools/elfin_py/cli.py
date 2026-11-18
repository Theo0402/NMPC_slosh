"""Common command line and session set-up for the bring-up tools.

Every tool takes exactly one of --dry-run, --mock or --arm (codebase plan §3.8 item 10):
  --dry-run  no connection; every command is printed and logged; the mock's simulation answers
  --mock     the local mock controller (python -m tools.elfin_mock), 127.0.0.1:10003/10004
  --arm      the real arm, at e03pro.yaml's address (ELFIN_IP overrides); Linux machine only
--assume-yes answers operator prompts with yes; refused with --arm.
--mirror-config (dry run only) makes the simulated controller report e03pro.yaml's limits, frames and
joint ranges, so that a rehearsal of an arm run takes the arm run's path (tools/elfin_mock/sim.py).

Started by the workbench (SLOSH_WORKBENCH=1), questions and the stop button go through the channel in
tools/elfin_py/channel.py instead of the terminal (codebase plan §12).

During a session, stdout and stderr never raise: once the workbench has died nobody reads the pipe, and a
print on a stop or cleanup path must not end that path before the deceleration or GrpDisable has gone out
(pre-hardware review S5). A failed trial-log write is a stop request, and the tool ends with LogIncomplete
(review S8).

SIGTERM and SIGHUP (the terminal closed, `kill`, a shutdown; SIGBREAK on Windows) do what Ctrl-C does, so the
deceleration and the exit sequence still run; and a tool started with SIGINT ignored (a background start from a
script) gets Python's own handler back, or the stop button and Ctrl-C would do nothing (review S13).

--mock talks to the local mock only: --ip must be a loopback address. --arm runs only against the owner's
config/arm/confirmed.yaml and config/arm/e03pro.yaml (review S14).
"""
import argparse
import contextlib
import io
import ipaddress
import json
import os
import select
import signal
import sys
import threading

from tools.elfin_py import channel, config, link, motion, protocol, triallog

STOP_SIGNALS = ("SIGTERM", "SIGHUP", "SIGBREAK")      # those this platform has
received = {"signal": None}                            # the last one, for the log ("stopped: SIGTERM")


def like_ctrl_c(signum, frame):
    """A termination signal does what Ctrl-C does at that moment: KeyboardInterrupt (the tool's finally blocks run);
    the servo ladder's stop flag while it streams; nothing while the exit sequence runs, which ignores SIGINT."""
    received["signal"] = signal.Signals(signum).name
    handler = signal.getsignal(signal.SIGINT)
    if callable(handler):
        handler(signum, frame)


def install_stop_signals():
    """Returns the handlers it replaced, for restore_signals(). Main thread only (signal.signal's rule)."""
    old = {}
    if threading.current_thread() is not threading.main_thread():
        return old
    if not callable(signal.getsignal(signal.SIGINT)):    # SIG_IGN inherited, or SIG_DFL (ends without cleanup)
        old[signal.SIGINT] = signal.signal(signal.SIGINT, signal.default_int_handler)
    for name in STOP_SIGNALS:
        sig = getattr(signal, name, None)
        if sig is not None:
            old[sig] = signal.signal(sig, like_ctrl_c)
    return old


def restore_signals(old):
    for sig, handler in old.items():
        if handler is not None:
            signal.signal(sig, handler)


def _loopback(host):
    if host.strip().lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host.strip()).is_loopback
    except ValueError:
        return False


def _same_file(a, b):
    return os.path.normcase(os.path.realpath(a)) == os.path.normcase(os.path.realpath(b))


class LogIncomplete(RuntimeError):
    """The trial log could not be written to the end; the run was stopped (pre-hardware review S8)."""


class SafeStream:
    """stdout or stderr, wrapped so that a failing write is dropped instead of raised (pre-hardware review S5).
    After the first failure nothing more is tried; the trial log keeps the record."""

    def __init__(self, stream):
        self._s = stream
        self.failed = None

    def write(self, text):
        if self.failed is None:
            try:
                return self._s.write(text)
            except (OSError, ValueError) as exc:      # BrokenPipeError on Linux, OSError 22 on Windows, closed file
                self.failed = exc
        return len(text)

    def flush(self):
        if self.failed is None:
            try:
                self._s.flush()
            except (OSError, ValueError) as exc:
                self.failed = exc

    def __getattr__(self, name):
        return getattr(self._s, name)


def parser(description):
    ap = argparse.ArgumentParser(description=description, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", dest="mode", action="store_const", const="dry-run")
    mode.add_argument("--mock", dest="mode", action="store_const", const="mock")
    mode.add_argument("--arm", dest="mode", action="store_const", const="arm")
    ap.add_argument("--ip", help="override the address (default: 127.0.0.1 for --mock, e03pro.yaml or ELFIN_IP for --arm)")
    ap.add_argument("--cmd-port", type=int)
    ap.add_argument("--state-port", type=int)
    ap.add_argument("--data-dir", help="where trials are written (default: slosh/data, or SLOSH_DATA)")
    ap.add_argument("--assume-yes", action="store_true", help="answer prompts with yes (--dry-run and --mock only)")
    ap.add_argument("--mirror-config", action="store_true",
                    help="dry run only: the simulated controller reports e03pro.yaml's limits and frames")
    ap.add_argument("--note", default="", help="operator note for the log header")
    ap.add_argument("--confirmed", default=str(config.CONFIRMED), help=argparse.SUPPRESS)
    ap.add_argument("--arm-config", default=str(config.ARM), help=argparse.SUPPRESS)
    return ap


def _terminal_answer(question, timeout):
    """The typed answer, or None if none came within `timeout` s. The timeout needs select() on stdin, so it
    applies on Linux (where arm runs happen) and not in a Windows terminal."""
    prompt = f"{question} Type 'yes' to continue: "
    if timeout is not None and os.name == "posix":
        try:
            print(prompt + f"(no answer within {timeout:g} s is taken as no) ", end="", flush=True)
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            return sys.stdin.readline().strip().lower() if ready else None
        except (OSError, ValueError, io.UnsupportedOperation):
            pass                                   # stdin is not a real file: ask without a timeout
    return input(prompt).strip().lower()


def make_ask(args, log, chan=None):
    def ask(question, timeout=None):
        """True only for 'yes'. With `timeout` (s), no answer in time is taken as no (pre-hardware review S9)."""
        if args.assume_yes:
            log.event(4, f"prompt auto-answered yes ({args.mode}): {question}")
            print(f"{question} [auto-yes]", flush=True)
            if chan is not None:
                chan.emit("auto_answer", question=question, answer="yes")
            return True
        if chan is not None:
            answer = chan.ask(question, timeout=timeout)
            if answer is None and timeout is not None and not chan.stopped:
                log.event(4, f"prompt (workbench): {question} -> no answer within {timeout:g} s, taken as no")
                print(f"NO ANSWER within {timeout:g} s: taken as no", flush=True)
                return False
            answer = (answer or "").strip().lower()
            log.event(4, f"prompt (workbench): {question} -> {answer!r}")
            return answer == "yes"
        answer = _terminal_answer(question, timeout)
        if answer is None:
            log.event(4, f"prompt: {question} -> no answer within {timeout:g} s, taken as no")
            print(f"\nNO ANSWER within {timeout:g} s: taken as no", flush=True)
            return False
        log.event(4, f"prompt: {question} -> {answer!r}")
        return answer == "yes"
    return ask


def workbench_block(environ=None):
    """The workbench's header block (codebase plan §5, §12): stage and coefficient snapshot."""
    env = os.environ if environ is None else environ
    block = {"stage": env.get("SLOSH_WB_STAGE")}
    path = env.get("SLOSH_WB_COEFFS")
    if path:
        with open(path, encoding="utf-8") as f:
            snap = json.load(f)
        block["coeff_version"] = snap.get("version")
        block["coeffs"] = snap.get("values")
    return block


@contextlib.contextmanager
def session(args, tool, dataset, first_run_note, mock_arm=None, connect=True):
    """Load config, open the trial log, connect. Yields (arm, log, confirmed, arm_cfg, ask)."""
    if args.mode == "arm" and args.assume_yes:
        sys.exit("--assume-yes is refused with --arm")
    if args.mirror_config and args.mode != "dry-run":
        sys.exit("--mirror-config is for --dry-run only (for --mock, start the mock with --mirror-config)")
    if args.mode == "mock" and args.ip and not _loopback(args.ip):
        sys.exit(f"--mock talks to the local mock controller only, and --ip {args.ip} is not a loopback address. "
                 "To reach the arm, use --arm, which runs the arm's checks.")
    if args.mode == "arm":
        for flag, given, owners in (("--confirmed", args.confirmed, config.CONFIRMED),
                                    ("--arm-config", args.arm_config, config.ARM)):
            if not _same_file(given, owners):
                sys.exit(f"{flag} {given} is refused with --arm: the arm runs only against {owners} "
                         "(codebase plan §3.8 item 5; only the owner edits confirmed.yaml)")
    out, err = sys.stdout, sys.stderr
    sys.stdout, sys.stderr = SafeStream(out), SafeStream(err)      # before the channel, which writes to stdout
    received["signal"] = None
    old_signals = install_stop_signals()
    try:
        with _session(args, tool, dataset, first_run_note, mock_arm, connect) as yielded:
            yield yielded
    finally:
        restore_signals(old_signals)
        sys.stdout, sys.stderr = out, err


@contextlib.contextmanager
def _session(args, tool, dataset, first_run_note, mock_arm, connect):
    chan = channel.Channel() if channel.active() else None
    args.channel = chan                 # the jog tool reads its commands from it
    confirmed, arm_cfg, texts = config.load(args.confirmed, args.arm_config)
    addr = arm_cfg["address"]
    if args.mode == "arm":
        ip = args.ip or os.environ.get("ELFIN_IP") or addr["ip"]
    else:
        ip = args.ip or "127.0.0.1"
    cmd_port = args.cmd_port or addr["cmd_port"]
    state_port = args.state_port or addr["state_port"]
    if args.mode == "dry-run" and args.mirror_config and mock_arm is None:
        from tools.elfin_mock.sim import MockArm, MockConfig, mirror_config
        mock_arm = MockArm(MockConfig(**mirror_config(arm_cfg)))
    extra = {"operator_note": args.note, "address": [ip, cmd_port, state_port], "confirmed": confirmed}
    if chan is not None:
        extra["workbench"] = workbench_block()
    log = triallog.TrialLog(tool, args.mode, dataset, texts, data_dir=args.data_dir, extra=extra)
    log.on_failure = lambda exc: channel.interrupt_main()      # a failed log write is a stop (review S8)
    print(f"trial {log.trial_id} ({args.mode}, dataset {dataset}) -> {log.path}", flush=True)
    if chan is not None:
        chan.on_stop = lambda reason: log.event(triallog.EV_MARKER, f"stop requested: {reason}")
        chan.emit("trial", trial_id=log.trial_id, path=str(log.path), tool=tool, mode=args.mode)
    arm = None
    try:
        arm = link.Arm(args.mode, log, ip=ip, cmd_port=cmd_port, state_port=state_port, mock_arm=mock_arm,
                       connect=connect)
        arm.joint_speed_limit = confirmed["joint_speed_deg_s"]   # measured during every WayPoint move (review S20)
        if args.mode == "arm":     # only once the host check has passed and the arm answered the connection
            print("FIRST RUN ON THE REAL ARM - watch:\n  " + first_run_note.replace("\n", "\n  "), flush=True)
        yield arm, log, confirmed, arm_cfg, make_ask(args, log, chan)
    except (link.Refused, motion.LimitRefused, protocol.RefusedCommand) as exc:
        log.event(triallog.EV_REFUSAL, str(exc))
        print(f"REFUSED: {exc}", file=sys.stderr, flush=True)
        raise
    except KeyboardInterrupt:
        if log.failed is not None:
            raise LogIncomplete(_incomplete(log)) from None
        log.event(triallog.EV_MARKER, f"stopped: {stop_reason(chan)}")
        print("STOPPED", file=sys.stderr, flush=True)
        raise
    finally:
        if arm is not None:
            arm.close()
        log.close()
        print(f"log written: {log.path}" + (" (INCOMPLETE)" if log.failed is not None else ""), flush=True)
    if log.failed is not None:     # the run ended before the stop request arrived, or SIGINT was ignored by then
        raise LogIncomplete(_incomplete(log))


def stop_reason(chan=None):
    """Why the tool was interrupted: the workbench's reason, a termination signal, or Ctrl-C."""
    if chan is not None and chan.stopped:
        return chan.stopped
    return received["signal"] or "Ctrl-C"


def _incomplete(log):
    msg = (f"the trial log could not be written ({log.failed!r}); the run was stopped and {log.path} is "
           "incomplete. Free disk space or fix the data disk before the next run.")
    print(f"TRIAL LOG INCOMPLETE: {msg}", file=sys.stderr, flush=True)
    return msg
