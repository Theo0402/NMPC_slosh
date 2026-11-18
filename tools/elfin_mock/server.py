"""Socket front of the mock: port 10003 (commands) and port 10004 (Datasheet).

Behaviour taken from the knowledge files (elfin_commands.md):
- a message is `Name,p1,...,;`; the reply ends with `,;` (PROTO p.3);
- the controller handles one complete message at a time; if several arrive together it
  processes the first valid one and drops the rest (PROTO p.1). The mock also treats
  anything that arrives before a reply has gone out as "arrived together", and counts it;
- the Datasheet is pushed without a request (PROTO p.7, p.185-188).

Mock choices, for testing our side: replies are split into fragments; reply delays,
late replies and dropped connections can be injected (MockConfig). Only one command client
is served at a time; a second connection is closed at once (whether the real controller
does so is [UNCLEAR], elfin_overview.md).
"""
import select
import socket
import threading
import time

from tools.elfin_mock.sim import MockArm, MockConfig

INCOMPLETE_TIMEOUT_S = 0.5   # an incomplete message gets an "invalid command" reply (PROTO p.1); timing is the mock's


class MockServer:
    def __init__(self, arm=None, host="127.0.0.1", cmd_port=10003, state_port=10004):
        self.arm = arm or MockArm(MockConfig())
        self.host = host
        self._stop = threading.Event()
        self.cmd_sock = self._listen(cmd_port)
        self.state_sock = self._listen(state_port)
        self.cmd_port = self.cmd_sock.getsockname()[1]
        self.state_port = self.state_sock.getsockname()[1]
        self._client = None
        self._threads = []
        self.connections_refused = 0

    def _listen(self, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, port))
        s.listen(4)
        s.settimeout(0.1)
        return s

    def start(self):
        for target in (self._accept_cmd, self._accept_state):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        for s in (self.cmd_sock, self.state_sock):
            s.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # ------------------------------------------------------------------ port 10003
    def _accept_cmd(self):
        while not self._stop.is_set():
            try:
                conn, _ = self.cmd_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            if self._client is not None:
                self.connections_refused += 1
                conn.close()
                continue
            self._client = conn
            t = threading.Thread(target=self._serve_cmd, args=(conn,), daemon=True)
            t.start()
            self._threads.append(t)

    def _serve_cmd(self, conn):
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(0.05)
        buf, first_byte_at, served = b"", None, 0
        try:
            while not self._stop.is_set():
                if b";" not in buf:
                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        if buf and time.monotonic() - first_byte_at > INCOMPLETE_TIMEOUT_S:
                            self.arm.stats["format_errors"] += 1
                            self._send(conn, "Unknown,Fail,990001,;")   # mock's "invalid command"
                            buf = b""
                        continue
                    if not chunk:
                        return
                    if not buf:
                        first_byte_at = time.monotonic()
                    buf += chunk
                    continue
                msg, _, rest = buf.partition(b";")
                text = (msg + b";").decode("ascii", "replace")
                if rest.strip():
                    self.arm.stats["dropped_extra_messages"] += 1   # arrived together: dropped (PROTO p.1)
                buf = b""
                reply = self.arm.handle(text)
                time.sleep(self.arm.reply_extra_delay(text))
                if self._pending(conn):
                    # the client sent more before our reply went out: one message at a time is violated
                    self.arm.stats["dropped_extra_messages"] += 1
                    self._drain(conn)
                self._send(conn, reply)
                served += 1
                if self.arm.cfg.drop_after_messages and served >= self.arm.cfg.drop_after_messages:
                    return
        except (ConnectionError, OSError):
            return
        finally:
            conn.close()
            self._client = None

    @staticmethod
    def _pending(conn):
        r, _, _ = select.select([conn], [], [], 0)
        return bool(r)

    @staticmethod
    def _drain(conn):
        try:
            while MockServer._pending(conn):
                if not conn.recv(4096):
                    break
        except OSError:
            pass

    def _send(self, conn, reply):
        data = reply.encode("ascii")
        if not self.arm.cfg.fragment_replies or len(data) < 4:
            conn.sendall(data)
            return
        rng = self.arm.rng
        cuts = sorted(rng.sample(range(1, len(data)), k=min(2, len(data) - 1)))
        prev = 0
        for c in cuts + [len(data)]:
            conn.sendall(data[prev:c])
            prev = c
            if c != len(data):
                time.sleep(0.0005)

    # ------------------------------------------------------------------ port 10004
    def _accept_state(self):
        while not self._stop.is_set():
            try:
                conn, _ = self.state_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            t = threading.Thread(target=self._serve_state, args=(conn,), daemon=True)
            t.start()
            self._threads.append(t)

    def _serve_state(self, conn):
        period = 1.0 / self.arm.cfg.datasheet_rate_hz
        next_t = time.monotonic()
        try:
            while not self._stop.is_set():
                conn.sendall(self.arm.datasheet_packet())
                next_t += period
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        except OSError:
            pass
        finally:
            conn.close()
