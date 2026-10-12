"""Cooperative signal handling and bounded, single-owner child shutdown.

Signals request a stop; they never interrupt an in-progress save or invalidate a
ROS context inside a callback. The mission supervisor owns sensor processes.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


class StopRequest:
    def __init__(self):
        self.requested = False
        self.signals = []
        self.previous = {}

    def handle(self, signum, frame):
        self.requested = True
        self.signals.append(signum)

    def __enter__(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            self.previous[sig] = signal.signal(sig, self.handle)
        return self

    def __exit__(self, *unused):
        for sig, handler in self.previous.items():
            signal.signal(sig, handler)


def atomic_text(path, text):
    """Replace one complete file; this is not a multi-file transaction."""
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                         prefix=path.name + '.', suffix='.tmp',
                                         delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def run_node(factory, finalize=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    with StopRequest() as stop:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = None
        try:
            node = factory()
            while not stop.requested and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.1)
        except ExternalShutdownException:
            if rclpy.ok():
                raise
        finally:
            try:
                if node is not None and finalize is not None:
                    finalize(node)
            finally:
                if node is not None:
                    node.destroy_node()
                rclpy.try_shutdown()


class ProcessStopper:
    """Idempotent stop of Popen children launched in their own sessions.

    Keep the Popen object as identity; do not target saved historical PIDs.
    Escalation is recorded and must not be called a clean exit.
    """
    def __init__(self):
        self.results = {}

    def stop(self, process, wait=20, terminate_wait=4, kill_wait=4):
        if process is None:
            return None
        if process in self.results:
            return self.results[process]
        started = time.monotonic()
        result = {'pid': process.pid, 'signals': [], 'escalated': False}
        self.results[process] = result
        for sig, timeout in ((signal.SIGINT, wait),
                             (signal.SIGTERM, terminate_wait),
                             (signal.SIGKILL, kill_wait)):
            if process.poll() is not None:
                break
            try:
                os.killpg(process.pid, sig)
                result['signals'].append(signal.Signals(sig).name)
                result['escalated'] |= sig != signal.SIGINT
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                continue
        result.update(exit_code=process.poll(), elapsed_s=time.monotonic()-started)
        if result['exit_code'] is None:
            raise RuntimeError('Owned child survived bounded shutdown: ' + json.dumps(result))
        return result
