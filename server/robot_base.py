from __future__ import annotations

import base64
import io
import time
import threading
from abc import ABC, abstractmethod


class _TaskInfo:
    """Internal record for a background task."""

    def __init__(self, name, thread, stop_event):
        self.name = name
        self.thread = thread
        self.stop_event = stop_event
        self.started_at = time.time()
        self.stopped_at = None
        self.error = None  # str if the task raised an exception

    @property
    def status(self):
        if self.thread.is_alive():
            return 'running'
        if self.error:
            return 'error'
        return 'stopped'


class RobotBase(ABC):
    """Abstract robot contract used by generic MCP/REPL runtime.

    Subclasses implement the abstract methods for robot-specific hardware.
    Universal runtime capabilities (task management, control loops, image
    capture) are provided here so every robot gets them for free.
    """

    def __init__(self) -> None:
        self._pending_images: list[str] = []
        self._tasks: dict[str, _TaskInfo] = {}

    # ── Abstract: subclasses must implement ──────────────────────────

    @abstractmethod
    def connect(self) -> None:
        """Initialize robot-side resources and connections."""

    @abstractmethod
    def close(self) -> None:
        """Release resources. Must be safe to call multiple times."""

    @abstractmethod
    def snapshot(self, device_name: str) -> dict:
        """Return structured snapshot payload for the given device."""

    @abstractmethod
    def _device_docs(self) -> str:
        """Return robot-specific hardware documentation (markdown).

        Called by describe() as part of the template method.
        Should include: connection status, device list, device-specific
        API reference (motors, cameras, sensors, etc.).
        """

    # ── Template method: introspection ───────────────────────────────

    def describe(self) -> str:
        """Return full documentation for LLM/operator consumption.

        Combines robot-specific hardware docs with universal runtime API.
        """
        return self._device_docs() + "\n" + self._runtime_api_docs()

    def _runtime_api_docs(self) -> str:
        """Return documentation for universal runtime capabilities."""
        lines = []
        lines.append("## Runtime API")
        lines.append("")
        lines.append("### Control Loops")
        lines.append("  robot.run(fn, duration=2.0, hz=30)")
        lines.append("    Calls fn(robot) at target rate for duration seconds. Blocking.")
        lines.append("    Good for short experiments (< 30s). For longer, use background tasks.")
        lines.append("")
        lines.append("### Background Tasks")
        lines.append("  robot.start_task(name, fn)                 → start fn(stop_event) in background")
        lines.append("  robot.stop_task(name, timeout=3.0)         → signal stop and wait")
        lines.append("  robot.stop_all_tasks()                     → stop everything")
        lines.append("  robot.task_status(name)                    → {'status', 'started_at', 'error', ...}")
        lines.append("  robot.list_tasks()                         → formatted status of all tasks")
        lines.append("")
        lines.append("  fn receives a threading.Event. Check stop.is_set() in your loop.")
        lines.append("  Use try/finally to return motors to a safe position on exit.")
        lines.append("")
        lines.append("### Visualization")
        lines.append("  robot.show(fig)                            → stashes matplotlib figure as PNG")
        lines.append("    Images from show() are returned to you through MCP after execute() completes.")
        lines.append("    Multiple show() calls → multiple images returned.")
        lines.append("")
        return '\n'.join(lines)

    # ── Image capture ────────────────────────────────────────────────

    def show(self, fig) -> None:
        """Render a matplotlib figure to PNG and queue it for REPL retrieval."""
        buf = io.BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight", dpi=100)
        buf.seek(0)
        self._pending_images.append(base64.b64encode(buf.read()).decode("ascii"))

    def _drain_images(self) -> list[str]:
        """Pop all images queued by show()."""
        images = self._pending_images
        self._pending_images = []
        return images

    # ── Control loop ─────────────────────────────────────────────────

    def run(self, fn, duration=1.0, hz=30):
        """Run fn(robot) at target rate for duration seconds. Blocking.

        Args:
            fn: callable taking (robot,) — can read sensors, set motors, etc.
            duration: seconds to run (float)
            hz: target loop rate (float)
        """
        period = 1.0 / hz
        t_start = time.time()
        while True:
            t_loop = time.time()
            elapsed = t_loop - t_start
            if elapsed >= duration:
                break
            fn(self)
            dt = time.time() - t_loop
            sleep_time = period - dt
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ── Background task management ───────────────────────────────────

    def start_task(self, name, fn):
        """Start a background task.

        Args:
            name: unique name for the task (e.g. 'track_face')
            fn: callable taking (stop_event,) — should check stop_event.is_set()
                in its loop and return when set. Use try/finally to return
                motors to a safe state on exit.
        """
        existing = self._tasks.get(name)
        if existing and existing.thread.is_alive():
            raise RuntimeError(f"Task '{name}' is already running. Stop it first with robot.stop_task('{name}').")

        stop_event = threading.Event()
        task_info = None  # forward reference for the closure

        def runner():
            try:
                fn(stop_event)
            except Exception as e:
                if task_info is not None:
                    task_info.error = f"{type(e).__name__}: {e}"
            finally:
                if task_info is not None:
                    task_info.stopped_at = time.time()

        thread = threading.Thread(target=runner, name=f"task-{name}", daemon=True)
        task_info = _TaskInfo(name, thread, stop_event)
        self._tasks[name] = task_info
        thread.start()
        return name

    def stop_task(self, name, timeout=3.0):
        """Stop a background task by name.

        Sets the stop event and waits for the thread to finish.
        Returns True if task was stopped, False if not found or already stopped.
        """
        task = self._tasks.get(name)
        if not task:
            raise ValueError(f"No task named '{name}'. Running: {[n for n, t in self._tasks.items() if t.thread.is_alive()]}")
        if not task.thread.is_alive():
            return False
        task.stop_event.set()
        task.thread.join(timeout=timeout)
        if task.thread.is_alive():
            return False
        return True

    def stop_all_tasks(self, timeout=3.0):
        """Stop all running background tasks. Returns list of names stopped."""
        running = [(n, t) for n, t in self._tasks.items() if t.thread.is_alive()]
        for name, task in running:
            task.stop_event.set()
        stopped = []
        for name, task in running:
            task.thread.join(timeout=timeout)
            if not task.thread.is_alive():
                stopped.append(name)
        return stopped

    def task_status(self, name=None):
        """Get status of a task or all tasks."""
        def _info(task):
            result = {
                'status': task.status,
                'started_at': task.started_at,
                'running_for': f"{time.time() - task.started_at:.1f}s" if task.thread.is_alive() else None,
            }
            if task.stopped_at:
                result['stopped_at'] = task.stopped_at
            if task.error:
                result['error'] = task.error
            return result

        if name is not None:
            task = self._tasks.get(name)
            if not task:
                raise ValueError(f"No task named '{name}'.")
            return _info(task)

        return {n: _info(t) for n, t in self._tasks.items()}

    def list_tasks(self):
        """List all tasks with their status. Returns a formatted string."""
        if not self._tasks:
            return "No tasks."
        lines = []
        for name, task in self._tasks.items():
            duration = time.time() - task.started_at
            line = f"  {name}: {task.status}"
            if task.thread.is_alive():
                line += f" (running {duration:.0f}s)"
            if task.error:
                line += f" — {task.error}"
            lines.append(line)
        return '\n'.join(lines)
