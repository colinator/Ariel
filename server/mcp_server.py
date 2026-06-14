#!/usr/bin/env python3
"""MCP server for Ariel.

Exposes robot hardware to LLMs via eleven tools:
  - health()                  — server + REPL health check (roundtrip ping)
  - describe()                — discover available devices
  - execute(code)             — run Python in a persistent REPL with `robot` object
  - snapshot(device)          — grab latest frame/state (lock-free observation plane)
  - shell(command)            — run shell commands (pip install, git clone, etc.)
  - save_module(name, code)   — save a Python module for import in the REPL
  - list_modules()            — list all saved modules
  - read_module(name)         — read a module's source code
  - read_log(lines)           — tail the REPL log, lock-free (works while REPL is busy)
  - interrupt()               — SIGINT the REPL to abort a running execute (lock-free)
  - peek()                    — lock-free status: exec-in-flight, joint state, cam freshness

Requires hardware_server.py running in a separate terminal.

Usage:
    python scripts/mcp_server.py

Then connect an MCP client to http://127.0.0.1:8750/mcp (streamable HTTP).
"""

import atexit
import ast
import base64
import signal
import json
import keyword
import os
import re
import struct
import subprocess
import sys
import asyncio
import select
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from mcp.server.fastmcp import FastMCP
import mcp.types as types

REPO_ROOT = Path(__file__).resolve().parent.parent
ROBOT_CONF = REPO_ROOT / "robot.conf"
IMAGE_CACHE_DIR = REPO_ROOT / "image_cache"
IMAGE_RETURN_MODE = os.getenv("ARIEL_IMAGE_RETURN_MODE", "file").strip().lower()
# On-device REPL log path (must match repl_server.py's default). Read lock-free by
# the read_log tool so it works even while the REPL lock is held by a long call.
REPL_LOG_PATH = Path(os.getenv("ARIEL_REPL_LOG", "/tmp/ariel_repl.log"))
SERVER_ROOT = Path(os.getenv("ARIEL_SERVER_ROOT", str(REPO_ROOT))).resolve()
CLIENT_ROOT_RAW = os.getenv("ARIEL_CLIENT_ROOT")
CLIENT_ROOT = Path(CLIENT_ROOT_RAW).expanduser().resolve() if CLIENT_ROOT_RAW else None
VALID_IMAGE_RETURN_MODES = {
    "inline",
    "file",
    "html",
    "inline+file",
    "inline+html",
}

# When true (default), tools advertise an output schema via structured_output.
# Some MCP clients (e.g. Claude Code as of March 2025) fail to render inline
# images when a tool has an output schema.  Set ARIEL_STRUCTURED_OUTPUT=false
# to disable output schemas on image-returning tools as a workaround.
# See: https://github.com/anthropics/claude-code/issues/31208
STRUCTURED_OUTPUT = os.getenv("ARIEL_STRUCTURED_OUTPUT", "false").strip().lower() not in ("0", "false", "no")

if IMAGE_RETURN_MODE not in VALID_IMAGE_RETURN_MODES:
    raise RuntimeError(
        f"Invalid ARIEL_IMAGE_RETURN_MODE={IMAGE_RETURN_MODE!r}. "
        f"Expected one of: {sorted(VALID_IMAGE_RETURN_MODES)}"
    )

if IMAGE_RETURN_MODE in {"file", "inline+file"}:
    if CLIENT_ROOT is None:
        raise RuntimeError(
            "ARIEL_IMAGE_RETURN_MODE requires client-visible path mapping. "
            "Set ARIEL_CLIENT_ROOT (and optionally ARIEL_SERVER_ROOT)."
        )
    try:
        IMAGE_CACHE_DIR.resolve().relative_to(SERVER_ROOT)
    except ValueError as exc:
        raise RuntimeError(
            f"ARIEL_SERVER_ROOT={str(SERVER_ROOT)!r} does not contain image cache "
            f"{str(IMAGE_CACHE_DIR.resolve())!r}"
        ) from exc


# --- REPL subprocess management ---

class ReplConnection:
    """Manages the REPL subprocess lifecycle and communication."""

    def __init__(self):
        self._proc = None
        self._ready = False
        self._lock = asyncio.Lock()
        self._startup_timeout_s = 15.0
        self._exec_timeout_s = float(os.getenv("ARIEL_EXEC_TIMEOUT_S", "120"))
        self._busy = False  # True while an exec is in flight; lock-free read for interrupt
        self._just_restarted = False  # set on auto-respawn, cleared after warning

    def _start(self):
        """Launch a fresh REPL subprocess."""
        self._ready = False
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            self._proc.wait(timeout=3)

        self._proc = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "server.repl_server",
                "--robot-conf",
                str(ROBOT_CONF),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(REPO_ROOT),
        )

        # Wait for ready signal
        ready = self._read_message(timeout=self._startup_timeout_s)
        if not ready or ready.get('status') != 'ready':
            stderr = self._read_stderr_snippet()
            self.stop()
            if stderr:
                raise RuntimeError(f"REPL failed to start: {ready}\n{stderr}")
            raise RuntimeError(f"REPL failed to start: {ready}")
        self._ready = True
        print(f"REPL subprocess started (pid={self._proc.pid})")

    def _read_message(self, timeout=None):
        """Read a length-prefixed JSON message from the REPL.

        Uses os.read() on the raw fd instead of BufferedReader.read() to
        avoid a subtle incompatibility: select() checks the OS pipe buffer,
        but BufferedReader maintains its own internal buffer.  Data can sit
        in the BufferedReader buffer (invisible to select), causing select
        to report "not ready" and the read to time out even though the data
        has already arrived.
        """
        fd = self._proc.stdout.fileno()

        def _read_exact(n, deadline):
            buf = bytearray()
            while len(buf) < n:
                if deadline is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    r, _, _ = select.select([fd], [], [], remaining)
                    if not r:
                        return None
                chunk = os.read(fd, n - len(buf))
                if not chunk:
                    return None  # EOF — REPL died
                buf.extend(chunk)
            return bytes(buf)

        deadline = None if timeout is None else (time.monotonic() + timeout)

        header = _read_exact(4, deadline)
        if header is None:
            return None
        length = struct.unpack('>I', header)[0]
        data = _read_exact(length, deadline)
        if data is None:
            return None
        return json.loads(data.decode('utf-8'))

    def _write_message(self, obj):
        """Write a length-prefixed JSON message to the REPL."""
        data = json.dumps(obj).encode('utf-8')
        self._proc.stdin.write(struct.pack('>I', len(data)))
        self._proc.stdin.write(data)
        self._proc.stdin.flush()

    def _read_stderr_snippet(self, timeout_s=0.2, max_bytes=4096):
        """Best-effort read of child stderr for startup diagnostics."""
        if not self._proc or not self._proc.stderr:
            return ""
        try:
            r, _, _ = select.select([self._proc.stderr], [], [], timeout_s)
            if not r:
                return ""
            data = self._proc.stderr.read(max_bytes)
            if not data:
                return ""
            return data.decode("utf-8", errors="replace").strip()
        except Exception:
            return ""

    def _is_alive(self):
        return self._proc is not None and self._proc.poll() is None

    async def send_code(self, code, *, silent=False):
        """Send code to the REPL and return the response.

        If the REPL has crashed, respawns it and returns an error message.
        Thread-safe via asyncio lock.

        When silent=True the REPL suppresses session-log output.  Used for
        internal read-only operations (describe, snapshot) so the log only
        shows LLM-authored code.
        """
        async with self._lock:
            # Ensure REPL is running
            restarted = False
            if not self._is_alive():
                try:
                    print("REPL not running, starting...")
                    await asyncio.get_event_loop().run_in_executor(None, self._start)
                    restarted = True
                    self._just_restarted = True
                except Exception as e:
                    return {
                        'stdout': '',
                        'stderr': f'Failed to start REPL: {e}',
                        'result': None,
                        'images': [],
                    }

            # Send code and get response
            try:
                def _exchange():
                    msg = {'code': code}
                    if silent:
                        msg['silent'] = True
                    self._write_message(msg)
                    return self._read_message(timeout=self._exec_timeout_s)

                self._busy = True
                try:
                    response = await asyncio.get_event_loop().run_in_executor(None, _exchange)
                finally:
                    self._busy = False

                if response is None:
                    # REPL died during execution
                    stderr = ''
                    try:
                        stderr = self._proc.stderr.read(4096).decode('utf-8', errors='replace')
                    except Exception:
                        pass
                    self._proc = None
                    self._ready = False
                    return {
                        'stdout': '',
                        'stderr': f'REPL crashed during execution. Restarting...\n{stderr}',
                        'result': None,
                        'images': [],
                    }

                # Inject restart warning so the LLM knows state was lost
                if restarted and not silent:
                    warning = "[REPL restarted — all previous state (variables, imports, functions) has been lost]\n"
                    response['stderr'] = warning + (response.get('stderr') or '')

                return response

            except (BrokenPipeError, OSError) as e:
                self._proc = None
                self._ready = False
                return {
                    'stdout': '',
                    'stderr': f'REPL connection lost: {e}. Will restart on next call.',
                    'result': None,
                    'images': [],
                }

    async def check_freshness(self):
        """Query the REPL for its freshness state (user-defined globals, robot attrs)."""
        async with self._lock:
            if not self._is_alive():
                return {'fresh': True, 'extra_globals': [], 'extra_robot_attrs': []}
            try:
                def _exchange():
                    self._write_message({'check_freshness': True})
                    return self._read_message(timeout=5.0)
                result = await asyncio.get_event_loop().run_in_executor(None, _exchange)
                return result or {'fresh': True, 'extra_globals': [], 'extra_robot_attrs': []}
            except Exception:
                return {'fresh': True, 'extra_globals': [], 'extra_robot_attrs': []}

    def stop(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            self._proc.wait(timeout=3)
            print("REPL subprocess stopped.")
        self._ready = False


# --- MCP Server ---

repl = ReplConnection()


class ObservationClient:
    """Lock-free, read-only tap into the robot's sensor/state streams.

    Builds the robot's ``observation_devices()`` in the MCP-server process (its own
    ZMQ context + subscriber graphs), independent of the REPL and its lock. Fully
    optional: if init fails it stays empty and snapshot/peek degrade gracefully.
    """

    def __init__(self):
        self.devices = {}      # name -> proxy (start/stop/observe)
        self.kinds = {}        # name -> kind str
        self._ctx = None
        self.error = None

    def start(self):
        try:
            import roboflex.transport.zmq as rzmq
            from server.robot_loader import load_robot_class
            cls = load_robot_class(str(ROBOT_CONF))
            self._ctx = rzmq.ZMQContext()
            for name, kind, proxy in cls.observation_devices(self._ctx):
                proxy.start()
                self.devices[name] = proxy
                self.kinds[name] = kind
            print(f"Observation client started: {list(self.devices)}")
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            print(f"WARNING: observation client init failed: {self.error}")

    def observe(self, name, **kw):
        proxy = self.devices.get(name)
        return proxy.observe(**kw) if proxy is not None else None

    def stop(self):
        for p in self.devices.values():
            try:
                p.stop()
            except Exception:
                pass


observation = ObservationClient()
atexit.register(repl.stop)


MODULES_DIR = REPO_ROOT / "modules"
MODULE_NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _sanitize_module_name(name: str) -> str:
    return "".join(c for c in name.strip() if c.isalnum() or c == "_")


def _image_mode_guidance() -> str:
    if IMAGE_RETURN_MODE == "inline":
        return (
            "Images are returned as inline MCP ImageContent. Use your visual understanding "
            "on those image blocks directly."
        )
    if IMAGE_RETURN_MODE == "file":
        return (
            "Images are not returned inline. When an operation produces an image, you will "
            "receive a client-visible filesystem path to the cached image file instead."
        )
    if IMAGE_RETURN_MODE == "html":
        return (
            "Images are not returned inline. When an operation produces an image, you will "
            "receive an HTTP URL to the cached image instead."
        )
    if IMAGE_RETURN_MODE == "inline+file":
        return (
            "Images are returned both inline as MCP ImageContent and as cached filesystem "
            "paths. The filesystem path is rewritten to be client-visible. Prefer the inline "
            "image when your client can actually inspect it."
        )
    return (
        "Images are returned both inline as MCP ImageContent and as cached HTTP URLs. "
        "Prefer the inline image when your client can actually inspect it."
    )


def _build_server_instructions() -> str:
    return f"""\
You are connected to a physical robot via Ariel, an MCP-based REPL for robot control.

You have eleven tools:

1. **describe** — Returns a full description of the robot's hardware, the `robot` API,
available libraries, and usage tips.

2. **health** — Check server liveness and REPL responsiveness (roundtrip ping).
Also reports whether the REPL is fresh (no user state) or has accumulated state
from a previous session (lists extra globals and robot attributes).

3. **execute** — Run Python code in a persistent REPL on the robot. The REPL has
a pre-loaded `robot` object for hardware access. State persists across calls —
imports, variables, and functions stick around. This is your primary tool.

4. **snapshot** — Quick peek at any device by name. For cameras this returns an image;
for motors it returns structured state text.

5. **shell** — Run a shell command on the robot. Runs in the same virtualenv as the REPL.
Use this only for `pip install`. If you want to do something else in the shell, ask the user first.

6. **save_module** — Save a Python module to the modules/ directory. The module
becomes importable in the REPL immediately (e.g. `from vision import detect_face`).
Use this to extract reusable functions from your REPL experiments.

7. **list_modules** — List all saved modules with their descriptions.

8. **read_module** — Read a module's full source code. Use this before modifying
an existing module so you can see what's already there.

9. **read_log** — Read the tail of the REPL session log WITHOUT taking the REPL lock.
One of two tools (with `interrupt`) that work while the REPL is busy or wedged. Use it
to recover the output of an `execute` that exceeded the configured exec timeout (the
call keeps running and its output still lands in the log), or to see what the REPL last
did during a hang. The log records COMPLETED calls (buffered, not streamed), so it is
not a live progress feed — for live progress, have a background task write to a global
you poll.

10. **interrupt** — Lock-free Ctrl-C: SIGINT the REPL to abort a running `execute`.
Fires only when an exec is in flight; the interrupted code raises KeyboardInterrupt and
is caught gracefully, so REPL state (variables, imports, modules) survives — no restart.
May not unstick a hung C-extension call (restart for those); stops the arm where it
froze, so re-home afterward. Emergency brake — prefer a background task's stop event for
graceful stops.

11. **peek** — Lock-free status: is an `execute` in flight + REPL pid, the latest joint/
servo state, and how fresh each camera stream is. Reads the observation plane (a second
subscriber tap, independent of the REPL lock), so it works mid-move / while busy. Use it
to watch a long op, or to read joint angles without disturbing a running call.

Operating principles:
- You are the PROGRAMMER, not the low-latency controller. Write code, run it, observe, iterate.
- Establish session state before assuming prior context. Use `health` to learn whether the REPL is fresh or stale.
- Inspect the robot description before using hardware. Use `describe` to learn device names, APIs, limits, and conventions.
- For perception tasks, inspect relevant sensors before writing control logic.
- For real-time behavior, write on-device loops with `robot.run()` or `robot.start_task()` instead of polling tools repeatedly.
- Keep data on-device. Analyze with numpy/scipy, return summaries and plots via `robot.show(fig)` rather than pulling large arrays over MCP.
- Move motors smoothly. Prefer interpolation over large instantaneous position jumps, using `robot.run()` or a short stepped loop when appropriate.
- For long-running behaviors, manage background tasks explicitly. Use `robot.list_tasks()` or `robot.task_status()` when behavior is not as expected.
- A single `execute` is capped at the configured exec timeout (default 120s, `ARIEL_EXEC_TIMEOUT_S`); longer calls time out while the REPL keeps running (its output then only reaches the log). For anything longer, launch a background task and poll a global with short `execute` calls — use `read_log` to recover output or watch the REPL, and `interrupt` to abort a runaway.
- Reads are lock-free: `snapshot` and `peek` tap the sensor/state streams directly (a second subscriber, independent of the REPL lock), so you can observe camera frames and joint angles even while the REPL is busy moving. So to watch a long motion: kick it off (background task), then `peek`/`snapshot` to see progress.
- The REPL runs in a separate process. If your code crashes, the hardware is fine. Fix the code and re-run.
- If the REPL crashes and restarts, you will see a warning in stderr:
  "[REPL restarted — all previous state (variables, imports, functions) has been lost]".
  When you see this, re-import and re-define anything you need.

Image return mode:
- {_image_mode_guidance()}

Workflow defaults:
- Look before acting when the task depends on current scene state.
- Check existing modules before writing new ones.
- Build up incrementally in the REPL, then save stable logic into modules.
"""


def _describe_image_mode_note() -> str:
    return (
        "\n## Image Return Mode\n"
        f"- Current mode: `{IMAGE_RETURN_MODE}`\n"
        f"- {_image_mode_guidance()}\n"
    )


def _client_visible_path(path: Path) -> str:
    try:
        rel = path.resolve().relative_to(SERVER_ROOT)
    except ValueError as exc:
        raise RuntimeError(
            f"Path {str(path)!r} is outside ARIEL_SERVER_ROOT={str(SERVER_ROOT)!r}"
        ) from exc
    if CLIENT_ROOT is None:
        raise RuntimeError("ARIEL_CLIENT_ROOT is required for file image mode")
    return str((CLIENT_ROOT / rel).resolve())


def _validate_module_name(name: str) -> tuple[str | None, str | None]:
    """Validate module name strictly.

    Returns:
        (safe_name, error_message). Exactly one is None.
    """
    raw = name.strip()
    if not raw:
        return None, "no module name provided"
    if not MODULE_NAME_RE.fullmatch(raw):
        suggestion = _sanitize_module_name(raw)
        msg = (
            "name must be a valid Python identifier "
            "(letters, digits, underscores; cannot start with a digit)"
        )
        if suggestion and MODULE_NAME_RE.fullmatch(suggestion) and not keyword.iskeyword(suggestion):
            msg += f". Suggested: '{suggestion}'"
        return None, msg
    if keyword.iskeyword(raw):
        return None, f"name '{raw}' is a Python keyword and cannot be used as a module name"
    return raw, None

SERVER_INSTRUCTIONS = _build_server_instructions()

mcp_app = FastMCP(
    "ariel",
    instructions=SERVER_INSTRUCTIONS,
    host="0.0.0.0",
    port=8750,
)


# --- Image cache: save images to disk and serve over HTTP ---

IMAGE_CACHE_DIR.mkdir(exist_ok=True)


def _save_image_to_cache(img_b64: str, prefix: str, mime: str) -> str:
    """Decode a base64 image, save to image_cache/, return the filename."""
    ext = "jpg" if "jpeg" in mime else "png"
    ts = time.strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}_{ts}.{ext}"
    path = IMAGE_CACHE_DIR / filename
    path.write_bytes(base64.b64decode(img_b64))
    return filename


def _image_url(filename: str, request=None) -> str:
    """Build the URL for a cached image."""
    if request is not None:
        base = str(request.base_url).rstrip("/")
    else:
        base = "http://localhost:8750"
    return f"{base}/images/{filename}"


def _emit_cached_image_refs(
    blocks: list[types.TextContent | types.ImageContent],
    *,
    filename: str,
    mime: str,
) -> None:
    if IMAGE_RETURN_MODE in {"file", "inline+file"}:
        blocks.append(types.TextContent(
            type="text",
            text=json.dumps({
                "image_file": _client_visible_path(IMAGE_CACHE_DIR / filename),
                "mime_type": mime,
            }),
        ))
    if IMAGE_RETURN_MODE in {"html", "inline+html"}:
        blocks.append(types.TextContent(
            type="text",
            text=json.dumps({
                "image_url": _image_url(filename),
                "mime_type": mime,
            }),
        ))


def _emit_image_blocks(
    blocks: list[types.TextContent | types.ImageContent],
    *,
    img_b64: str,
    prefix: str,
    mime: str,
) -> None:
    filename = _save_image_to_cache(img_b64, prefix, mime)
    if IMAGE_RETURN_MODE in {"inline", "inline+file", "inline+html"}:
        blocks.append(types.ImageContent(
            type="image",
            data=img_b64,
            mimeType=mime,
        ))
    _emit_cached_image_refs(blocks, filename=filename, mime=mime)


@mcp_app.custom_route("/images/{path:path}", methods=["GET"])
async def serve_images(request):
    from starlette.responses import FileResponse, JSONResponse

    rel_path = request.path_params["path"]
    file_path = IMAGE_CACHE_DIR / rel_path

    # Prevent path traversal
    try:
        file_path.resolve().relative_to(IMAGE_CACHE_DIR.resolve())
    except ValueError:
        return JSONResponse({"error": "forbidden"}, status_code=403)

    if not file_path.is_file():
        return JSONResponse({"error": "not found"}, status_code=404)

    return FileResponse(file_path)


@mcp_app.tool(
    name="health",
    description=(
        "Health check for MCP server and REPL. Reports server liveness, "
        "REPL process status, and performs a roundtrip ping to verify the "
        "REPL is responsive (~10s timeout). Starts the REPL if needed. "
        "Also reports repl_fresh (true if no user state exists) and "
        "repl_state (list of extra globals and robot attrs) when state is present."
    ),
)
async def health() -> dict:
    pid = None
    if repl._proc is not None and repl._proc.poll() is None:
        pid = repl._proc.pid

    # Roundtrip ping to verify REPL is actually responsive
    repl_ok = False
    repl_error = None
    try:
        saved = repl._exec_timeout_s
        repl._exec_timeout_s = 10.0
        try:
            response = await repl.send_code("'ok'", silent=True)
        finally:
            repl._exec_timeout_s = saved
        stderr = (response.get("stderr") or "").strip()
        repl_ok = response.get("result") == "'ok'" and not bool(stderr)
        if not repl_ok:
            repl_error = stderr or "unexpected response"
        # Update pid after send_code (REPL may have just started)
        if repl._proc is not None and repl._proc.poll() is None:
            pid = repl._proc.pid
    except Exception as e:
        repl_error = str(e)

    # Check REPL freshness (has user-defined state?)
    freshness = {}
    if repl_ok:
        try:
            freshness = await repl.check_freshness()
        except Exception:
            pass

    result = {
        "server_ok": True,
        "repl_pid": pid,
        "repl_ok": repl_ok,
        **({"repl_error": repl_error} if repl_error else {}),
    }
    if freshness:
        result["repl_fresh"] = freshness.get("fresh", True)
        if not freshness.get("fresh", True):
            result["repl_state"] = {
                "extra_globals": freshness.get("extra_globals", []),
                "extra_robot_attrs": freshness.get("extra_robot_attrs", []),
            }
    return result


@mcp_app.tool(
    name="describe",
    description=(
        "Discover the robot and how to control it. Returns a comprehensive guide "
        "including: physical hardware description, device names and capabilities, "
        "the full `robot` API with examples, available Python libraries, and tips."
    ),
)
async def describe() -> str:
    response = await repl.send_code("print(robot.describe())", silent=True)
    if response['stderr']:
        return f"Error: {response['stderr']}"
    return response['stdout'].rstrip() + _describe_image_mode_note() + "\n"


@mcp_app.tool(
    name="execute",
    description=(
        "Execute Python code in a persistent REPL session running on the robot. "
        "A `robot` object is pre-loaded with access to all hardware. "
        "State persists across calls — imports, variables, and function definitions "
        "stick around, so you can build up complex logic incrementally.\n\n"
        "The REPL returns: stdout, the repr of the last expression (like a Python "
        "interactive session), stderr (with tracebacks on error), and any images "
        "produced by robot.show(fig). Image return format depends on the server's "
        f"configured image mode ({IMAGE_RETURN_MODE})."
    ),
    **({"structured_output": False} if not STRUCTURED_OUTPUT else {}),
)
async def execute(code: str) -> list[types.TextContent | types.ImageContent]:
    if not code.strip():
        return [types.TextContent(type="text", text="Error: no code provided")]

    response = await repl.send_code(code)

    blocks = []

    # Text output: combine stdout, result, and stderr
    text_parts = []
    if response['stdout']:
        text_parts.append(response['stdout'].rstrip())
    if response['result'] is not None:
        text_parts.append(f"=> {response['result']}")
    if response['stderr']:
        text_parts.append(f"STDERR:\n{response['stderr'].rstrip()}")

    if text_parts:
        blocks.append(types.TextContent(type="text", text='\n'.join(text_parts)))

    # Images from robot.show()
    for i, img_b64 in enumerate(response.get('images', [])):
        _emit_image_blocks(
            blocks,
            img_b64=img_b64,
            prefix=f"execute_{i}",
            mime="image/png",
        )

    if not blocks:
        blocks.append(types.TextContent(type="text", text="(no output)"))

    return blocks


@mcp_app.tool(
    name="snapshot",
    description=(
        "Grab the latest frame/state from a named device — a quick peek without writing "
        "code. Served lock-free from the observation plane (a direct subscriber tap) when "
        "the device is in the robot's observation manifest, so it works even while the "
        "REPL is busy; otherwise it falls back to the REPL. For cameras: returns a live, "
        "downscaled JPEG (tagged '[lock-free]' with frame age). For state: latest joint/"
        "servo values. Device names are listed in the describe output."
    ),
    **({"structured_output": False} if not STRUCTURED_OUTPUT else {}),
)
async def snapshot(device: str) -> list[types.TextContent | types.ImageContent]:
    if not device:
        return [types.TextContent(type="text", text="Error: no device name provided")]

    # Lock-free path: read directly from the observation client (no REPL lock).
    if device in observation.devices:
        payload = None
        try:
            payload = await asyncio.get_event_loop().run_in_executor(
                None, lambda: observation.observe(device))
        except Exception as exc:
            print(f"observation snapshot('{device}') failed, falling back to REPL: {exc!r}")
        if payload is not None:
            blocks: list[types.TextContent | types.ImageContent] = []
            if payload.get("type") == "image" and payload.get("data"):
                _emit_image_blocks(blocks, img_b64=payload["data"],
                                   prefix=f"snapshot_{device}", mime="image/jpeg")
            ts = payload.get("timestamp")
            age = f"  ({time.time() - ts:.2f}s old)" if ts else ""
            blocks.append(types.TextContent(
                type="text",
                text=(payload.get("summary") or f"snapshot from '{device}'") + age + "  [lock-free]"))
            return blocks

    snapshot_code = f"""
_dev = {device!r}
_r = robot.snapshot(_dev)
if _r.get("type") == "image":
    if _r.get("data"):
        robot._pending_images.append(_r["data"])
    _fmt = _r.get("format", "jpeg")
    print(f"__imgfmt:{{_fmt}}")
    print(_r.get("summary") or f"snapshot image from '{{_dev}}'")
elif _r.get("type") == "state":
    print(_r.get("summary") or str(_r.get("state", _r)))
else:
    print(str(_r))
"""
    response = await repl.send_code(snapshot_code, silent=True)

    # Detect image format from stdout marker
    img_mime = "image/png"
    stdout = response.get('stdout', '')
    if '__imgfmt:jpeg' in stdout:
        img_mime = "image/jpeg"
        stdout = stdout.replace('__imgfmt:jpeg\n', '').replace('__imgfmt:jpeg', '')
    elif '__imgfmt:png' in stdout:
        stdout = stdout.replace('__imgfmt:png\n', '').replace('__imgfmt:png', '')
    response['stdout'] = stdout

    blocks = []
    for img_b64 in response.get('images', []):
        _emit_image_blocks(
            blocks,
            img_b64=img_b64,
            prefix=f"snapshot_{device}",
            mime=img_mime,
        )
    if response['stdout']:
        blocks.append(types.TextContent(type="text", text=response['stdout'].rstrip()))
    if response['stderr']:
        blocks.append(types.TextContent(type="text", text=f"Error: {response['stderr'].rstrip()}"))
    if not blocks:
        blocks.append(types.TextContent(type="text", text="(no output)"))
    return blocks


@mcp_app.tool(
    name="shell",
    description=(
        "Run a shell command on the robot. Runs in the same environment as the REPL "
        "(same virtualenv, same working directory). Use this only for pip install.\n\n"
        "The command runs synchronously — long operations (e.g. pip install) will block "
        "until complete. Timeout is 5 minutes.\n\n"
        "Note: pip install works (packages persist across restarts). "
        "apt/system packages are not available (no root access). "
        "If you want to use the shell for something else, ask the user first. "
        "If you need a system package, ask the user to add it to the Docker image."
    ),
)
async def shell(command: str) -> str:
    if not command.strip():
        return "Error: no command provided"

    # Run shell command through the REPL so it inherits the venv
    shell_code = (
        "import subprocess as _sp\n"
        f"_r = _sp.run({command!r}, shell=True, capture_output=True, text=True, timeout=300)\n"
        "_out = _r.stdout\n"
        "if _r.stderr:\n"
        "    _out += '\\nSTDERR:\\n' + _r.stderr\n"
        "_out += f'\\n(exit code {_r.returncode})'\n"
        "print(_out)"
    )
    response = await repl.send_code(shell_code)

    parts = []
    if response['stdout']:
        parts.append(response['stdout'].rstrip())
    if response['stderr']:
        parts.append(f"REPL error:\n{response['stderr'].rstrip()}")
    return '\n'.join(parts) if parts else "(no output)"


@mcp_app.tool(
    name="save_module",
    description=(
        "Save a Python module to the modules/ directory. The module becomes immediately "
        "importable in the REPL (e.g. `from vision import detect_face`).\n\n"
        "Use this to extract reusable functions from your REPL experiments into "
        "clean, importable modules. Overwrites if the module already exists.\n\n"
        "Parameters:\n"
        "- name: Module name (e.g. 'vision', 'motion', 'utils'). Must be a valid "
        "Python identifier (letters, digits, underscores).\n"
        "- code: The module source code.\n\n"
        "Include a module-level docstring; make the first line a concise summary, "
        "since list_modules returns that line as doc_summary.\n\n"
        "Returns structured metadata including whether it was created or overwritten, "
        "line counts, file path, and a human-readable summary."
    ),
)
async def save_module(name: str, code: str) -> dict:
    safe_name, name_error = _validate_module_name(name)
    if name_error:
        return {
            "ok": False,
            "error": name_error,
            "summary": f"Error: {name_error}",
        }
    if not code.strip():
        return {
            "ok": False,
            "error": "no code provided",
            "summary": "Error: no code provided",
        }

    try:
        ast.parse(code, filename=f"{safe_name}.py")
    except SyntaxError as exc:
        location = f"line {exc.lineno}"
        if exc.offset is not None:
            location += f", column {exc.offset}"
        detail = exc.msg or "invalid syntax"
        error = f"syntax error in module code at {location}: {detail}"
        return {
            "ok": False,
            "error": error,
            "summary": f"Error: {error}",
        }

    MODULES_DIR.mkdir(exist_ok=True)
    filepath = MODULES_DIR / f"{safe_name}.py"

    # Check if overwriting
    existed = filepath.exists()
    old_lines = len(filepath.read_text(encoding="utf-8").splitlines()) if existed else 0

    filepath.write_text(code + "\n", encoding="utf-8")
    new_lines = len(code.splitlines())
    file_bytes = filepath.stat().st_size

    # Ensure modules/ is on sys.path in the REPL
    await repl.send_code(
        f"import sys; _p = {str(MODULES_DIR.resolve())!r}\n"
        f"if _p not in sys.path: sys.path.insert(0, _p)",
        silent=True,
    )

    import_response = await repl.send_code(
        f"import importlib, sys, traceback\n"
        f"importlib.invalidate_caches()\n"
        f"try:\n"
        f"    if {safe_name!r} in sys.modules:\n"
        f"        importlib.reload(sys.modules[{safe_name!r}])\n"
        f"        print('__module_action__:reloaded')\n"
        f"    else:\n"
        f"        importlib.import_module({safe_name!r})\n"
        f"        print('__module_action__:imported')\n"
        f"except Exception:\n"
        f"    print('__module_action__:error')\n"
        f"    traceback.print_exc()\n",
        silent=True,
    )
    import_stdout = import_response.get("stdout", "")
    import_stderr = import_response.get("stderr", "")
    import_action = "created"
    if "__module_action__:reloaded" in import_stdout:
        import_action = "reloaded"
    elif "__module_action__:imported" in import_stdout:
        import_action = "imported"
    elif "__module_action__:error" in import_stdout:
        import_action = "error"

    import_stdout = (
        import_stdout
        .replace("__module_action__:reloaded\n", "")
        .replace("__module_action__:imported\n", "")
        .replace("__module_action__:error\n", "")
        .replace("__module_action__:reloaded", "")
        .replace("__module_action__:imported", "")
        .replace("__module_action__:error", "")
        .strip()
    )

    import_error = "\n".join(part for part in (import_stdout, import_stderr.strip()) if part).strip()
    imported = import_action == "imported"
    reloaded = import_action == "reloaded"

    action = "overwrote" if existed else "created"
    summary = (
        f"{action.capitalize()} {filepath} "
        f"(was {old_lines} lines, now {new_lines} lines, {file_bytes} bytes). "
        f"Import with: from {safe_name} import ..."
    )
    if imported:
        summary += " Imported successfully in the REPL."
    elif reloaded:
        summary += " Reloaded existing import."
    elif import_error:
        summary += f" Import check failed: {import_error.splitlines()[-1]}"

    return {
        "ok": not bool(import_error),
        "name": safe_name,
        "path": str(filepath),
        "action": action,
        "created": not existed,
        "overwritten": existed,
        "old_lines": old_lines,
        "new_lines": new_lines,
        "bytes": file_bytes,
        "imported": imported,
        "reloaded": reloaded,
        **({"import_error": import_error} if import_error else {}),
        "import_hint": f"from {safe_name} import ...",
        "summary": summary,
    }


@mcp_app.tool(
    name="list_modules",
    description=(
        "List all saved Python modules in the modules/ directory. "
        "Returns structured metadata for each module: name, bytes, line count, and "
        "the first line of the module docstring when available. "
        "Use this to discover what's already been built before writing new code."
    ),
)
async def list_modules() -> dict:
    if not MODULES_DIR.exists():
        return {
            "ok": True,
            "count": 0,
            "modules": [],
            "summary": "No modules saved yet. Use save_module to create your first one.",
        }

    files = sorted(MODULES_DIR.glob("*.py"))
    if not files:
        return {
            "ok": True,
            "count": 0,
            "modules": [],
            "summary": "No modules saved yet. Use save_module to create your first one.",
        }

    modules = []
    for f in files:
        text = f.read_text(encoding="utf-8")
        line_count = len(text.splitlines())
        byte_count = f.stat().st_size

        # Extract module docstring robustly via AST.
        desc = None
        try:
            parsed = ast.parse(text)
            docstring = ast.get_docstring(parsed, clean=True)
            if docstring:
                desc = docstring.split("\n")[0].strip() or None
        except SyntaxError:
            desc = None

        modules.append(
            {
                "name": f.stem,
                "path": str(f),
                "lines": line_count,
                "bytes": byte_count,
                "doc_summary": desc,
            }
        )

    return {
        "ok": True,
        "count": len(files),
        "modules": modules,
        "summary": f"Modules ({len(files)}).",
    }


@mcp_app.tool(
    name="read_module",
    description=(
        "Read the full source code of a saved module. Use this before modifying "
        "an existing module so you can see what's there and make targeted changes.\n\n"
        "Parameters:\n"
        "- name: Module name (e.g. 'vision', 'motion'). No .py extension needed."
    ),
)
async def read_module(name: str) -> str:
    safe_name, name_error = _validate_module_name(name)
    if name_error:
        return f"Error: {name_error}"
    filepath = MODULES_DIR / f"{safe_name}.py"

    if not filepath.exists():
        available = [f.stem for f in MODULES_DIR.glob("*.py")] if MODULES_DIR.exists() else []
        return f"Module '{safe_name}' not found. Available: {available}"

    text = filepath.read_text()
    return f"# {safe_name}.py ({len(text.splitlines())} lines)\n\n{text}"


@mcp_app.tool(
    name="read_log",
    description=(
        "Read the tail of the REPL session log WITHOUT acquiring the REPL lock. "
        f"Returns the last `lines` lines of the on-device log at {REPL_LOG_PATH} "
        "(the stdout/result/stderr of past execute calls, prefixed 'out:' / '=>' / 'ERR:').\n\n"
        "One of two lock-free tools (with interrupt) that work while the REPL is busy or "
        "wedged — every other tool (execute, snapshot, shell, health) goes through the "
        "single REPL lock and blocks behind a long-running call. Use read_log to:\n"
        "  - recover the output of an execute() that exceeded the configured exec timeout (the work "
        "keeps running and its output still lands in the log), and\n"
        "  - inspect what the REPL last did when a call hangs.\n\n"
        "Note: the log is written when each execute COMPLETES (buffered, not streamed), "
        "so it shows finished calls, NOT live progress of an in-flight call. For live "
        "progress, have a background task write to a global you poll.\n\n"
        "Parameters:\n"
        "- lines: number of trailing log lines to return (default 100)."
    ),
    **({"structured_output": False} if not STRUCTURED_OUTPUT else {}),
)
async def read_log(lines: int = 100) -> str:
    path = REPL_LOG_PATH
    if not path.exists():
        return f"(no REPL log at {path})"
    try:
        # Lock-free tail: read only the trailing bytes, never touch the REPL.
        max_bytes = 65536
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            chunk = f.read().decode("utf-8", "replace")
        rows = chunk.splitlines()
        if size > max_bytes and rows:
            rows = rows[1:]  # drop a possibly-partial first line
        n = lines if (lines and lines > 0) else len(rows)
        tail = rows[-n:]
        return f"# {path} — last {len(tail)} of {len(rows)} recent lines\n" + "\n".join(tail)
    except Exception as exc:
        return f"Error reading log: {exc!r}"


@mcp_app.tool(
    name="interrupt",
    description=(
        "Send a Ctrl-C (SIGINT) to the REPL to abort a running execute() — lock-free, "
        "so it works while the REPL is busy. Fires only if an exec is actually in "
        "flight (otherwise a no-op, so an idle REPL is never disturbed). The interrupted "
        "code raises KeyboardInterrupt and is caught gracefully: the REPL and all its "
        "state (variables, imports, modules) survive — you get an error response, not a "
        "restart.\n\n"
        "Caveats: aborts pure-Python loops cleanly, but may NOT unstick a hung "
        "C-extension call (e.g. a wedged camera grab) — restart the REPL for those. If "
        "the arm was mid-move it stops where it froze, so command a safe pose afterward. "
        "The graceful way to stop a long job is a background task's stop event; this is "
        "the emergency brake."
    ),
    **({"structured_output": False} if not STRUCTURED_OUTPUT else {}),
)
async def interrupt() -> str:
    if not repl._is_alive():
        return "REPL is not running; nothing to interrupt."
    if not getattr(repl, "_busy", False):
        return "REPL is idle (no exec in flight); nothing to interrupt."
    try:
        repl._proc.send_signal(signal.SIGINT)
        return ("Sent SIGINT. The running execute() should abort with KeyboardInterrupt; "
                "REPL state is preserved. If it stays wedged (hung C call), restart the REPL.")
    except Exception as exc:
        return f"Failed to interrupt: {exc!r}"


@mcp_app.tool(
    name="peek",
    description=(
        "Lock-free status peek — works while the REPL is busy or wedged (does NOT take "
        "the REPL lock). Reports whether an execute is in flight + the REPL pid, the "
        "latest joint/servo state, and how fresh each camera stream is. State and cameras "
        "come from the observation plane (a second subscriber tap, independent of the "
        "REPL), so you can see joint angles and frame freshness even mid-move. For a full "
        "frame use snapshot; to recover a busy call's output use read_log."
    ),
    **({"structured_output": False} if not STRUCTURED_OUTPUT else {}),
)
async def peek() -> str:
    lines = []
    alive = repl._is_alive()
    pid = repl._proc.pid if (alive and repl._proc is not None) else None
    lines.append(f"REPL: {'alive' if alive else 'down'}  pid={pid}  "
                 f"exec_in_flight={getattr(repl, '_busy', False)}")
    try:
        sp = observation.observe("servo_state") if "servo_state" in observation.devices else None
    except Exception as exc:
        sp = None
        lines.append(f"state: read error {exc!r}")
    if sp:
        ts = sp.get("timestamp")
        age = f"{time.time() - ts:.2f}s old" if ts else "n/a"
        lines.append(f"state: {sp.get('summary')}  ({age})")
    elif observation.error:
        lines.append(f"state: observation client unavailable ({observation.error})")
    for name, kind in observation.kinds.items():
        if kind == "camera":
            lr = getattr(observation.devices[name], "last_recv", None)
            fresh = f"{time.time() - lr:.2f}s old" if lr else "no frame yet"
            lines.append(f"camera '{name}': {fresh}")
    return "\n".join(lines)


# --- Entry point ---

if __name__ == "__main__":
    def _boot_repl():
        try:
            print("Starting REPL subprocess at MCP startup...")
            repl._start()
        except Exception as exc:
            print(f"WARNING: initial REPL start failed: {exc}")
        observation.start()

    threading.Thread(target=_boot_repl, daemon=True).start()
    mcp_app.run(transport="streamable-http")
