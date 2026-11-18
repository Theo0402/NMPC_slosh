"""The workbench's HTTP API and page, on 127.0.0.1 only (codebase plan §12). Standard library only.

GET  /                        the page
GET  /api/overview            stages with their status per mode, host, current run
GET  /api/stage/<id>          one stage: criteria, locks, runs, results, its coefficients
GET  /api/run?since=N         the current run: console, questions, live plot data
POST /api/run                 {stage, mode, note, assume_yes, options, override_reason}
POST /api/stop                {run_id}: the stop button; it always stops the current run
POST /api/answer              {id, text, run_id}: refused if run_id is not the current run's
POST /api/accept              {run_id, note, operator_checks}
POST /api/form                {stage, values, note, operator_checks}
GET  /api/coeffs              the editor's table and the versions
GET  /api/coeffs/diff?a=&b=   the diff between two versions
POST /api/coeffs              {changes, note}
POST /api/coeffs/revert       {to, note}
GET  /api/confirmed           confirmed.yaml, read-only view
POST /api/confirmed           {changes, phrase, note}   (the owner's typed confirmation)
GET  /api/audit               the audit log
GET  /api/points              saved paths (start, middle and end points)
POST /api/points              {name, points, note}
POST /api/cmd                 {name: jog|goto|path|keep|end, args, run_id}: to the running Arm control session
GET  /api/pastruns?source=    run folders, newest first (source: data | backup)
GET  /api/pastrun?name=&source=   one run folder: summary, console, questions, commands, joint plot
POST /api/pastrun/open        {name, source}: show the folder in the file manager
GET  /api/backup              backup folder, last backup, runs held, space
POST /api/backup/now          copy what is new (never deletes)
POST /api/backup/settings     {folder, auto}
POST /api/backup/restore      {name, overwrite}: back into data/; a difference needs overwrite (owner-confirmed)

Only this machine's own page may use the API (pre-hardware review S15). Every request must name this server in its
Host header (no DNS rebinding), and a browser's Origin, if sent, must be this page; requests a browser marks as
cross-site are refused. Every /api/ request must carry the header X-Slosh-Workbench: 1, and a POST must be JSON:
a page on another site cannot send either without a CORS preflight, which this server never answers.
"""
import http.server
import json
import os
import pathlib
import urllib.parse

from tools.workbench.bench import Refused

STATIC = pathlib.Path(__file__).resolve().parent / "static"
PAGE_HEADER = "X-Slosh-Workbench"           # the page sends it with every API request; other sites cannot
MAX_BODY = 4 * 1024 * 1024                  # the largest request body read (coefficient edits, saved paths)


def make_handler(bench):
    class Handler(http.server.BaseHTTPRequestHandler):
        server_version = "slosh-workbench"

        def log_message(self, fmt, *args):          # keep the terminal for the operator
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body, default=str).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype + ("; charset=utf-8" if ctype.startswith("text") or
                                                      ctype == "application/json" else ""))
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _raw_body(self):
            """Read the whole body, also of a request about to be refused: a socket closed with unread data makes
            Windows reset the connection, and the client may then never see the refusal."""
            n = int(self.headers.get("Content-Length") or 0)
            if n > MAX_BODY:
                raise ValueError(f"the body is larger than {MAX_BODY} bytes")
            return self.rfile.read(n) if n > 0 else b""

        def _guard(self, fn):
            try:
                self._send(200, fn())
            except Refused as exc:
                self._send(409, {"refused": str(exc)})
            except (KeyError, ValueError, TypeError) as exc:
                self._send(400, {"error": f"{type(exc).__name__}: {exc}"})
            except Exception as exc:            # anything else: an answer the page can show, never a dropped connection
                self._send(500, {"error": f"{type(exc).__name__}: {exc}"})

        def _foreign(self, api, post):
            """Why this request is refused as coming from somewhere other than this machine's page, or None."""
            port = self.server.server_address[1]
            hosts = {f"127.0.0.1:{port}", f"localhost:{port}", f"[::1]:{port}"}
            if (self.headers.get("Host") or "").strip().lower() not in hosts:
                return "the Host header does not name this workbench"
            origin = self.headers.get("Origin")
            if origin is not None and origin.strip().lower() not in {f"http://{h}" for h in hosts}:
                return f"requests from {origin} are refused"
            if (self.headers.get("Sec-Fetch-Site") or "").strip().lower() == "cross-site":
                return "cross-site requests are refused"
            if api and self.headers.get(PAGE_HEADER) != "1":
                return f"API requests need the {PAGE_HEADER} header that the workbench's page sends"
            if post and (self.headers.get("Content-Type") or "").split(";")[0].strip().lower() != "application/json":
                return "a POST must be application/json"
            return None

        def do_GET(self):
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query)
            why = self._foreign(u.path.startswith("/api/"), post=False)
            if why:
                return self._send(403, {"error": why})
            if u.path in ("/", "/index.html"):
                return self._send(200, (STATIC / "index.html").read_bytes(), "text/html")
            if u.path == "/api/overview":
                return self._guard(bench.overview)
            if u.path.startswith("/api/stage/"):
                sid = u.path.rsplit("/", 1)[-1]
                if sid not in bench.reg.stages:
                    return self._send(404, {"error": f"no stage {sid}"})
                return self._guard(lambda: bench.stage_view(sid))
            if u.path == "/api/run":
                since = int((q.get("since") or ["0"])[0])
                return self._guard(lambda: bench.run.view(since) if bench.run else None)
            if u.path == "/api/coeffs":
                return self._guard(lambda: dict(bench.coeffs.table(), versions=bench.coeffs.versions()))
            if u.path == "/api/coeffs/diff":
                return self._guard(lambda: bench.coeffs.diff(int(q["a"][0]), int(q["b"][0])))
            if u.path == "/api/confirmed":
                return self._guard(bench.confirmed_view)
            if u.path == "/api/audit":
                return self._guard(bench.status.audit_log)
            if u.path == "/api/pastruns":
                return self._guard(lambda: bench.past_runs((q.get("source") or ["data"])[0]))
            if u.path == "/api/pastrun":
                return self._guard(lambda: bench.past_run(q["name"][0], (q.get("source") or ["data"])[0]))
            if u.path == "/api/backup":
                return self._guard(bench.backup_view)
            if u.path == "/api/points":
                return self._guard(bench.paths)
            self._send(404, {"error": "not found"})

        def do_POST(self):
            u = urllib.parse.urlparse(self.path)
            try:
                raw = self._raw_body()
            except ValueError as exc:
                return self._send(413, {"error": str(exc)})
            why = self._foreign(True, post=True)
            if why:
                return self._send(403, {"error": why})
            try:
                b = json.loads(raw.decode("utf-8")) if raw else {}
            except ValueError:
                return self._send(400, {"error": "the body must be JSON"})
            if not isinstance(b, dict):
                return self._send(400, {"error": "the body must be a JSON object"})
            routes = {
                "/api/run": lambda: {"run_id": bench.start(b["stage"], b["mode"], b.get("note", ""),
                                                           bool(b.get("assume_yes")), b.get("options") or {},
                                                           b.get("override_reason"), bool(b.get("practice")))},
                "/api/stop": lambda: {"stop_sent": bench.stop(b.get("run_id"))},
                "/api/answer": lambda: bench.answer(b["id"], b["text"], b.get("run_id")) or {"ok": True},
                "/api/accept": lambda: bench.accept(b["run_id"], b.get("note", ""), b.get("operator_checks") or {}),
                "/api/form": lambda: bench.submit_form(b["stage"], b.get("values") or {}, b.get("note", ""),
                                                       b.get("operator_checks") or {}, b.get("override_reason")),
                "/api/coeffs": lambda: bench.edit_coeffs(b["changes"], b.get("note", "")),
                "/api/coeffs/revert": lambda: bench.revert_coeffs(int(b["to"]), b.get("note", "")),
                "/api/cmd": lambda: bench.command(b["name"], b.get("args") or {}, b.get("run_id")),
                "/api/pastrun/open": lambda: bench.open_run_folder(b["name"], b.get("source", "data")),
                "/api/backup/now": bench.backup_now,
                "/api/backup/settings": lambda: bench.backup_settings(b["folder"], bool(b.get("auto", True))),
                "/api/backup/restore": lambda: bench.restore_run(b["name"], bool(b.get("overwrite"))),
                "/api/points": lambda: bench.save_path(b.get("name", ""), b.get("points") or [], b.get("note", "")),
                "/api/confirmed": lambda: bench.change_confirmed(b.get("changes") or {}, b.get("phrase", ""),
                                                                 b.get("note", "")),
            }
            fn = routes.get(u.path)
            if fn is None:
                return self._send(404, {"error": "not found"})
            self._guard(fn)

    return Handler


class Server(http.server.ThreadingHTTPServer):
    # On Windows, SO_REUSEADDR lets a second workbench bind the same port and silently share it; two workbenches on
    # one data folder would write run folders and backups at once. Refuse instead: the second one fails to start.
    allow_reuse_address = os.name != "nt"


def serve(bench, host="127.0.0.1", port=8765):
    if host not in ("127.0.0.1", "localhost", "::1"):
        raise ValueError("the workbench listens on the local machine only")
    httpd = Server((host, port), make_handler(bench))
    httpd.daemon_threads = True
    return httpd
