#!/usr/bin/env python3
"""REPL subprocess — persistent Python session with `robot` pre-loaded.

Communicates with parent over stdin/stdout using length-prefixed JSON:
  - 4-byte big-endian length prefix + JSON payload, both directions
  - Request:  {"code": "..."}
  - Response: {"stdout": "...", "stderr": "...", "result": "...", "images": ["<b64>", ...]}

The parent launches this as a subprocess and writes/reads its stdin/stdout.
Stderr of this process is left for diagnostics (not part of the protocol).

Usage (standalone for testing):
    python scripts/repl_server.py

Then type length-prefixed JSON on stdin (or use test_repl.py).
"""

import io
import os
import sys
import ast
import json
import struct
import argparse
import datetime
import traceback
import contextlib
from pathlib import Path

# --- CRITICAL: Protect the protocol stream ---
# Save the real binary stdout for protocol messages, then redirect
# Python-level stdout to stderr. This prevents C extensions (like ZMQ)
# or stray print() calls from corrupting the length-prefixed protocol.
_protocol_out = os.fdopen(os.dup(sys.stdout.fileno()), 'wb')
_protocol_in = os.fdopen(os.dup(sys.stdin.fileno()), 'rb')

# Redirect Python stdout/stderr to the original stderr fd so that
# any C-level writes to fd 1 also go to stderr, not the protocol.
os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
sys.stdout = sys.stderr

# Experimental path: use stdio streams directly for protocol transport.
# _protocol_out = sys.stdout.buffer
# _protocol_in = sys.stdin.buffer

# --- REPL session log ---
# Watch live with: tail -f ${ARIEL_REPL_LOG:-/tmp/ariel_repl.log}
REPL_LOG = os.getenv("ARIEL_REPL_LOG", "/tmp/ariel_repl.log")
_log_file = None


def _rotate_log():
    """Rotate existing log file to ariel_repl_N.log (N = next available index)."""
    log_path = Path(REPL_LOG)
    if not log_path.exists() or log_path.stat().st_size == 0:
        return
    stem = log_path.stem       # "ariel_repl"
    suffix = log_path.suffix   # ".log"
    parent = log_path.parent
    n = 0
    while (parent / f"{stem}_{n}{suffix}").exists():
        n += 1
    log_path.rename(parent / f"{stem}_{n}{suffix}")


def _log(text):
    global _log_file
    if _log_file is None:
        _rotate_log()
        _log_file = open(REPL_LOG, 'a')
    ts = datetime.datetime.now().strftime('%H:%M:%S')
    _log_file.write(f"[{ts}] {text}\n")
    _log_file.flush()

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from server.robot_loader import RobotLoadError, load_robot_class


def read_message(stream):
    """Read a length-prefixed JSON message from a binary stream."""
    header = stream.read(4)
    if len(header) < 4:
        return None  # EOF
    length = struct.unpack('>I', header)[0]
    data = stream.read(length)
    if len(data) < length:
        return None  # truncated
    return json.loads(data.decode('utf-8'))


def write_message(stream, obj):
    """Write a length-prefixed JSON message to a binary stream."""
    data = json.dumps(obj).encode('utf-8')
    stream.write(struct.pack('>I', len(data)))
    stream.write(data)
    stream.flush()


def extract_last_expr(code):
    """Try to separate the last expression from the rest of the code.

    Returns (setup_code, expr_code) if the last statement is an expression,
    or (code, None) if it's not.
    """
    try:
        tree = ast.parse(code)
    except SyntaxError:
        return code, None

    if not tree.body:
        return code, None

    last = tree.body[-1]
    if isinstance(last, ast.Expr):
        # The last statement is an expression — split it out.
        # Use AST source coordinates instead of line-only splitting so this works
        # for one-line multi-statement code like: "import time; time.sleep(1.0)".
        if len(tree.body) == 1:
            return None, code.strip()

        if (
            hasattr(last, "lineno")
            and hasattr(last, "col_offset")
            and hasattr(last, "end_lineno")
            and hasattr(last, "end_col_offset")
        ):
            lines = code.splitlines(keepends=True)

            def _offset(lineno, col):
                return sum(len(l) for l in lines[: lineno - 1]) + col

            start = _offset(last.lineno, last.col_offset)
            end = _offset(last.end_lineno, last.end_col_offset)

            setup = code[:start].rstrip()
            expr = code[start:end].strip()
            return setup if setup else None, expr if expr else None

        # Fallback for Python versions without end position metadata.
        line_idx = last.lineno - 1
        lines = code.split('\n')
        setup = '\n'.join(lines[:line_idx])
        expr = '\n'.join(lines[line_idx:])
        return setup if setup.strip() else None, expr.strip()

    return code, None


_REPL_DEFAULT_GLOBALS = {'robot', '__builtins__'}


def check_repl_freshness(globals_dict):
    """Check whether the REPL has accumulated state beyond the defaults.

    Returns a dict with:
      - fresh: True if no user state exists
      - extra_globals: list of user-defined names in globals_dict
      - extra_robot_attrs: list of attrs added to robot beyond its class defaults
    """
    extra_globals = sorted(
        k for k in globals_dict
        if k not in _REPL_DEFAULT_GLOBALS and not k.startswith('_')
    )
    robot = globals_dict.get('robot')
    extra_robot_attrs = []
    if robot is not None:
        class_attrs = set(dir(type(robot)))
        # cameras/motors are set in __init__, not class-level — always present
        init_instance_attrs = {'cameras', 'motors'}
        extra_robot_attrs = sorted(
            k for k in vars(robot)
            if k not in class_attrs and k not in init_instance_attrs and not k.startswith('_')
        )
    fresh = not extra_globals and not extra_robot_attrs
    return {
        'fresh': fresh,
        'extra_globals': extra_globals,
        'extra_robot_attrs': extra_robot_attrs,
    }


def execute_code(code, globals_dict):
    """Execute code in the persistent namespace, capturing output.

    Returns dict with stdout, stderr, result, images.
    """
    stdout_buf = io.StringIO()
    stderr_buf = io.StringIO()
    result = None

    setup_code, expr_code = extract_last_expr(code)

    try:
        with contextlib.redirect_stdout(stdout_buf), contextlib.redirect_stderr(stderr_buf):
            if setup_code:
                exec(compile(setup_code, '<repl>', 'exec'), globals_dict)
            if expr_code:
                result = eval(compile(expr_code, '<repl>', 'eval'), globals_dict)
    except Exception:
        stderr_buf.write(traceback.format_exc())

    # Drain images from robot.show()
    robot = globals_dict.get('robot')
    images = []
    if robot and hasattr(robot, '_drain_images'):
        images = robot._drain_images()

    return {
        'stdout': stdout_buf.getvalue(),
        'stderr': stderr_buf.getvalue(),
        'result': repr(result) if result is not None else None,
        'images': images,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="REPL subprocess with dynamically loaded robot.")
    parser.add_argument(
        "--robot-conf",
        default="robot.conf",
        help="Path to robot config file containing module:Class spec",
    )
    args = parser.parse_args(argv)

    print(f"REPL log: tail -f {REPL_LOG}", file=sys.stderr)
    _log("=== REPL session started ===")

    # Set up the persistent namespace
    robot = None
    try:
        robot_cls = load_robot_class(args.robot_conf)
        robot = robot_cls()
        robot.connect()
        _log(f"Robot connected: {robot_cls.__module__}.{robot_cls.__name__}")
    except RobotLoadError as exc:
        msg = f"Robot config/load error: {exc}"
        _log(msg)
        write_message(_protocol_out, {"status": "error", "error": msg})
        return
    except Exception as exc:
        msg = f"Robot connect error: {type(exc).__name__}: {exc}"
        _log(msg)
        _log(traceback.format_exc())
        write_message(_protocol_out, {"status": "error", "error": msg})
        return

    globals_dict = {
        'robot': robot,
        '__builtins__': __builtins__,
    }

    # Signal ready
    write_message(_protocol_out, {'status': 'ready'})
    _log("Ready, waiting for code...")

    try:
        n = 0
        while True:
            msg = read_message(_protocol_in)
            if msg is None:
                break  # parent closed stdin

            # Handle freshness check (no code execution needed)
            if msg.get('check_freshness'):
                write_message(_protocol_out, check_repl_freshness(globals_dict))
                continue

            n += 1
            code = msg.get('code', '')
            silent = msg.get('silent', False)

            if not silent:
                _log(f"--- exec #{n} ---")
                for line in code.rstrip().split('\n'):
                    _log(f"  >>> {line}")

            response = execute_code(code, globals_dict)

            if not silent:
                if response['stdout']:
                    for line in response['stdout'].rstrip().split('\n'):
                        _log(f"  out: {line}")
                if response['stderr']:
                    for line in response['stderr'].rstrip().split('\n'):
                        _log(f"  ERR: {line}")
                if response['result'] is not None:
                    _log(f"  => {response['result']}")
                if response['images']:
                    for i, img in enumerate(response['images']):
                        kb = len(img) * 3 // 4 // 1024  # approx decoded size
                        _log(f"  [image {i+1}: ~{kb}KB PNG]")

            write_message(_protocol_out, response)
    finally:
        _log("=== REPL session ended ===")
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    main()
