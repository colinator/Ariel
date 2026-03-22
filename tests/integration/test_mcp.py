#!/usr/bin/env python3
"""Test the MCP server tools over streamable HTTP.

Requires:
    python -m robots.pantilt.hardware   (in another terminal)
    python -m server.mcp_server         (in another terminal)

Usage:
    python tests/integration/test_mcp.py [--output-dir DIR]

Snapshot images are written to --output-dir (default: /tmp/ariel_test_mcp/).
"""

import argparse
import base64
import json
import sys
import time
import uuid
from pathlib import Path
from urllib.request import urlopen, Request
from urllib.error import URLError

OUTPUT_DIR = Path(__file__).resolve().parent.parent.parent / "testresults"


class McpClient:
    """Minimal MCP streamable-HTTP client with session management."""

    def __init__(self, url):
        self.url = url
        self.session_id = None

    def _send(self, body, timeout=60):
        """Send a JSON-RPC message and parse the SSE/JSON response.

        Returns the parsed JSON-RPC response, or None for notifications (202).
        """
        data = json.dumps(body).encode("utf-8")
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json, text/event-stream",
        }
        if self.session_id:
            headers["Mcp-Session-Id"] = self.session_id

        req = Request(self.url, data=data, headers=headers)

        with urlopen(req, timeout=timeout) as resp:
            # Capture session id
            sid = resp.headers.get("Mcp-Session-Id")
            if sid:
                self.session_id = sid

            # Notifications get 202 with empty body
            if resp.status == 202:
                return None

            content_type = resp.headers.get("Content-Type", "")
            if "text/event-stream" in content_type:
                result = None
                for line in resp:
                    line = line.decode("utf-8").strip()
                    if line.startswith("data: "):
                        event_data = json.loads(line[6:])
                        if "result" in event_data or "error" in event_data:
                            result = event_data
                if result is None:
                    raise RuntimeError("No JSON-RPC response in SSE stream")
                return result
            else:
                raw = resp.read()
                if not raw:
                    return None
                return json.loads(raw.decode("utf-8"))

    def initialize(self):
        """Perform MCP initialize + initialized handshake."""
        resp = self._send({
            "jsonrpc": "2.0",
            "id": str(uuid.uuid4()),
            "method": "initialize",
            "params": {
                "protocolVersion": "2025-03-26",
                "capabilities": {},
                "clientInfo": {"name": "test_mcp", "version": "1.0"},
            },
        })
        # Send initialized notification (no id = notification, returns 202)
        self._send({
            "jsonrpc": "2.0",
            "method": "notifications/initialized",
        })
        return resp

    def call_tool(self, name, args=None):
        """Call an MCP tool and return the result content list."""
        resp = self._send({
            "jsonrpc": "2.0",
            "id": str(uuid.uuid4()),
            "method": "tools/call",
            "params": {
                "name": name,
                "arguments": args or {},
            },
        }, timeout=60)
        if "error" in resp:
            raise RuntimeError(f"MCP error: {resp['error']}")
        return resp["result"]["content"]


# Module-level client, initialized in main()
client: McpClient = None


def call_tool(name, args=None):
    """Convenience wrapper."""
    return client.call_tool(name, args)


def save_image(content_block, name, output_dir):
    """Save a base64 image content block to disk. Returns the path."""
    output_dir.mkdir(parents=True, exist_ok=True)
    img_data = base64.b64decode(content_block["data"])
    mime = content_block.get("mimeType", "image/png")
    ext = "jpg" if "jpeg" in mime else "png"
    path = output_dir / f"{name}.{ext}"
    path.write_bytes(img_data)
    return path


def get_text(content):
    """Extract concatenated text from content blocks."""
    return "\n".join(b["text"] for b in content if b["type"] == "text")


def get_images(content):
    """Extract image content blocks."""
    return [b for b in content if b["type"] == "image"]


# --- Tests ---

def test_health(output_dir):
    print("\n=== Test 1: health ===")
    content = call_tool("health")
    text = get_text(content)
    assert "server_ok" in text or "repl_ok" in text or "repl_pid" in text, \
        f"Expected health fields, got: {text[:200]}"
    print(f"  {text[:200]}")
    print("  PASS")


def test_describe(output_dir):
    print("\n=== Test 2: describe ===")
    content = call_tool("describe")
    text = get_text(content)
    assert "Camera" in text or "camera" in text, f"Expected camera info, got: {text[:200]}"
    assert "Motor" in text or "motor" in text, f"Expected motor info, got: {text[:200]}"
    assert "pan" in text, f"Expected 'pan' in describe output"
    assert "tilt" in text, f"Expected 'tilt' in describe output"
    print(f"  Got {len(text)} chars of description")
    print("  PASS")


def test_snapshot_camera(output_dir):
    print("\n=== Test 3: snapshot (camera) ===")
    content = call_tool("snapshot", {"device": "main"})
    images = get_images(content)
    assert len(images) >= 1, f"Expected at least 1 image, got {len(images)}"

    path = save_image(images[0], "snapshot_camera", output_dir)
    size_kb = path.stat().st_size / 1024
    print(f"  Image saved: {path} ({size_kb:.1f} KB)")
    assert size_kb > 1, f"Image suspiciously small: {size_kb:.1f} KB"

    text = get_text(content)
    if text:
        print(f"  Text: {text[:100]}")
    print("  PASS")


def test_snapshot_motor(output_dir):
    print("\n=== Test 4: snapshot (motor) ===")
    for motor_name in ["pan", "tilt"]:
        content = call_tool("snapshot", {"device": motor_name})
        text = get_text(content)
        assert text, f"Expected text for motor snapshot '{motor_name}'"
        print(f"  {motor_name}: {text[:100]}")
    print("  PASS")


def test_execute_expression(output_dir):
    print("\n=== Test 5: execute (expression) ===")
    content = call_tool("execute", {"code": "1 + 1"})
    text = get_text(content)
    assert "2" in text, f"Expected '2' in result, got: {text}"
    print(f"  {text.strip()}")
    print("  PASS")


def test_execute_print(output_dir):
    print("\n=== Test 6: execute (print) ===")
    content = call_tool("execute", {"code": "print('hello from mcp')"})
    text = get_text(content)
    assert "hello from mcp" in text, f"Expected 'hello from mcp', got: {text}"
    print(f"  {text.strip()}")
    print("  PASS")


def test_execute_persistent_state(output_dir):
    print("\n=== Test 7: execute (persistent state) ===")
    call_tool("execute", {"code": "_test_var = 99"})
    content = call_tool("execute", {"code": "_test_var * 2"})
    text = get_text(content)
    assert "198" in text, f"Expected '198', got: {text}"
    print(f"  {text.strip()}")
    print("  PASS")


def test_execute_error(output_dir):
    print("\n=== Test 8: execute (error) ===")
    content = call_tool("execute", {"code": "1 / 0"})
    text = get_text(content)
    assert "ZeroDivisionError" in text, f"Expected ZeroDivisionError, got: {text}"
    print(f"  {text[:80].strip()}...")
    print("  PASS")


def test_execute_robot_describe(output_dir):
    print("\n=== Test 9: execute robot.describe() ===")
    content = call_tool("execute", {"code": "robot.describe()"})
    text = get_text(content)
    assert "camera" in text.lower() or "motor" in text.lower(), \
        f"Expected hardware info, got: {text[:200]}"
    print(f"  Got {len(text)} chars")
    print("  PASS")


def test_execute_grab_frame(output_dir):
    print("\n=== Test 10: execute grab_frame ===")
    content = call_tool("execute", {"code": "robot.cameras['main'].grab_frame().shape"})
    text = get_text(content)
    assert "480" in text and "640" in text, f"Expected 640x480 shape, got: {text}"
    print(f"  {text.strip()}")
    print("  PASS")


def test_execute_show(output_dir):
    print("\n=== Test 11: execute robot.show() ===")
    code = """\
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots()
ax.plot([1,2,3], [1,4,9])
ax.set_title('MCP test')
robot.show(fig)
plt.close(fig)
'done'
"""
    content = call_tool("execute", {"code": code})
    images = get_images(content)
    assert len(images) >= 1, f"Expected at least 1 image from show(), got {len(images)}"

    path = save_image(images[0], "execute_show", output_dir)
    size_kb = path.stat().st_size / 1024
    print(f"  Plot saved: {path} ({size_kb:.1f} KB)")
    print("  PASS")


def test_shell(output_dir):
    print("\n=== Test 12: shell ===")
    content = call_tool("shell", {"command": "echo 'hello from shell'"})
    text = get_text(content)
    assert "hello from shell" in text, f"Expected echo output, got: {text}"
    print(f"  {text[:100].strip()}")
    print("  PASS")


def test_modules_lifecycle(output_dir):
    print("\n=== Test 13: save/list/read module ===")
    mod_name = "_test_module"
    mod_code = '"""Test module for MCP integration test."""\n\ndef greet(name):\n    return f"hello {name}"\n'

    # save
    content = call_tool("save_module", {"name": mod_name, "code": mod_code})
    text = get_text(content)
    print(f"  save: {text[:100]}")

    # list
    content = call_tool("list_modules")
    text = get_text(content)
    print(f"  list: {text[:100]}")

    # read
    content = call_tool("read_module", {"name": mod_name})
    text = get_text(content)
    assert "def greet" in text, f"Expected module source, got: {text[:100]}"
    print(f"  read: got {len(text)} chars")

    # use it from the REPL
    content = call_tool("execute", {"code": f"from {mod_name} import greet; greet('ariel')"})
    text = get_text(content)
    assert "hello ariel" in text, f"Expected 'hello ariel', got: {text}"
    print(f"  execute: {text.strip()}")

    # cleanup
    content = call_tool("shell", {"command": f"rm -f modules/{mod_name}.py"})
    print("  PASS")


def test_snapshot_camera_second(output_dir):
    """Take a second snapshot to verify images aren't stale/cached."""
    print("\n=== Test 14: snapshot (camera, second grab) ===")
    time.sleep(0.5)  # let at least a few frames arrive
    content = call_tool("snapshot", {"device": "main"})
    images = get_images(content)
    assert len(images) >= 1, f"Expected at least 1 image"

    path = save_image(images[0], "snapshot_camera_2", output_dir)
    size_kb = path.stat().st_size / 1024
    print(f"  Image saved: {path} ({size_kb:.1f} KB)")
    print("  PASS")


# --- Main ---

ALL_TESTS = [
    test_health,
    test_describe,
    test_snapshot_camera,
    test_snapshot_motor,
    test_execute_expression,
    test_execute_print,
    test_execute_persistent_state,
    test_execute_error,
    test_execute_robot_describe,
    test_execute_grab_frame,
    test_execute_show,
    test_shell,
    test_modules_lifecycle,
    test_snapshot_camera_second,
]


def main():
    global client

    default_url = "http://127.0.0.1:8750/mcp"
    parser = argparse.ArgumentParser(description="Test MCP server tools over HTTP.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=OUTPUT_DIR,
        help=f"Directory for snapshot images (default: {OUTPUT_DIR})",
    )
    parser.add_argument(
        "--url",
        default=default_url,
        help=f"MCP server URL (default: {default_url})",
    )
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output dir: {output_dir}")

    # Connect and initialize MCP session
    print(f"Connecting to MCP server at {args.url}...")
    try:
        client = McpClient(args.url)
        resp = client.initialize()
        print(f"Session: {client.session_id}")
        print(f"Server: {json.dumps(resp.get('result', {}).get('serverInfo', {}))}")
    except URLError as e:
        print(f"Cannot connect to MCP server at {args.url}: {e}")
        print("Is the server running? (python -m server.mcp_server)")
        sys.exit(1)

    passed = 0
    failed = 0
    for test_fn in ALL_TESTS:
        try:
            test_fn(output_dir)
            passed += 1
        except Exception as e:
            failed += 1
            print(f"\n!!! FAILED: {e}")
            import traceback
            traceback.print_exc()

    print(f"\n=== {passed} passed, {failed} failed, images in {output_dir} ===")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
