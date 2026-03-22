#!/usr/bin/env python3
"""Test the REPL subprocess by launching it and sending code snippets.

Requires: python -m robots.pantilt.hardware (in another terminal)

Usage:
    python tests/integration/test_repl.py
"""

import json
import struct
import subprocess
import sys
import time


def read_message(stream):
    """Read a length-prefixed JSON message from a binary stream."""
    header = stream.read(4)
    if len(header) < 4:
        return None
    length = struct.unpack('>I', header)[0]
    data = stream.read(length)
    if len(data) < length:
        return None
    return json.loads(data.decode('utf-8'))


def write_message(stream, obj):
    """Write a length-prefixed JSON message to a binary stream."""
    data = json.dumps(obj).encode('utf-8')
    stream.write(struct.pack('>I', len(data)))
    stream.write(data)
    stream.flush()


def send_code(proc, code):
    """Send code to the REPL and return the response."""
    write_message(proc.stdin, {'code': code})
    return read_message(proc.stdout)


def main():
    print("Launching REPL subprocess...")
    proc = subprocess.Popen(
        [sys.executable, "-m", "server.repl_server", "--robot-conf", "robot.conf"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    try:
        # Wait for ready signal
        ready = read_message(proc.stdout)
        print(f"REPL ready: {ready}")
        assert ready.get('status') == 'ready', f"Expected ready, got {ready}"

        # --- Test 1: Simple expression ---
        print("\n=== Test 1: simple expression ===")
        r = send_code(proc, "1 + 1")
        print(f"  result={r['result']}, stdout={r['stdout']!r}, stderr={r['stderr']!r}")
        assert r['result'] == '2', f"Expected '2', got {r['result']}"
        print("  PASS")

        # --- Test 2: Print statement ---
        print("\n=== Test 2: print statement ===")
        r = send_code(proc, "print('hello world')")
        print(f"  result={r['result']}, stdout={r['stdout']!r}")
        assert 'hello world' in r['stdout']
        print("  PASS")

        # --- Test 3: Persistent state ---
        print("\n=== Test 3: persistent state ===")
        send_code(proc, "x = 42")
        r = send_code(proc, "x * 2")
        print(f"  result={r['result']}")
        assert r['result'] == '84'
        print("  PASS")

        # --- Test 4: Import persistence ---
        print("\n=== Test 4: import persistence ===")
        send_code(proc, "import math")
        r = send_code(proc, "math.pi")
        print(f"  result={r['result']}")
        assert '3.14' in r['result']
        print("  PASS")

        # --- Test 5: Syntax error ---
        print("\n=== Test 5: syntax error ===")
        r = send_code(proc, "def foo(")
        print(f"  stderr={r['stderr'][:80]!r}...")
        assert 'SyntaxError' in r['stderr']
        print("  PASS")

        # --- Test 6: Runtime error ---
        print("\n=== Test 6: runtime error ===")
        r = send_code(proc, "1 / 0")
        print(f"  stderr={r['stderr'][:80]!r}...")
        assert 'ZeroDivisionError' in r['stderr']
        print("  PASS")

        # --- Test 7: robot.describe() ---
        print("\n=== Test 7: robot.describe() ===")
        r = send_code(proc, "robot.describe()")
        print(f"  result={r['result'][:100]}")
        assert 'cameras' in r['result']
        assert 'motors' in r['result']
        print("  PASS")

        # --- Test 8: grab_frame shape ---
        print("\n=== Test 8: grab_frame ===")
        r = send_code(proc, "robot.cameras['main'].grab_frame().shape")
        print(f"  result={r['result']}")
        assert '480' in r['result'] and '640' in r['result']
        print("  PASS")

        # --- Test 9: motor read ---
        print("\n=== Test 9: motor position ===")
        r = send_code(proc, "robot.motors['pan'].get_position()")
        print(f"  result={r['result']}")
        assert r['result'] is not None
        print("  PASS")

        # --- Test 10: robot.show() ---
        print("\n=== Test 10: robot.show() ===")
        r = send_code(proc, """
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots()
ax.plot([1,2,3], [1,4,9])
robot.show(fig)
plt.close(fig)
'done'
""")
        print(f"  result={r['result']}, images={len(r['images'])}")
        assert len(r['images']) == 1
        assert len(r['images'][0]) > 100
        print("  PASS")

        # --- Test 11: Multi-line with print + expression ---
        print("\n=== Test 11: multi-line ===")
        r = send_code(proc, """
a = 10
b = 20
print(f"sum = {a + b}")
a * b
""")
        print(f"  stdout={r['stdout']!r}, result={r['result']}")
        assert 'sum = 30' in r['stdout']
        assert r['result'] == '200'
        print("  PASS")

        print("\n=== ALL TESTS PASSED ===")

    except Exception as e:
        print(f"\n!!! TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        print("REPL subprocess terminated.")


if __name__ == "__main__":
    main()
