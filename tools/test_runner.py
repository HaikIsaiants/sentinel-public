import io
import struct
import subprocess
import sys
import threading

import run_scenario
from sentinel.v1 import sentinel_pb2


class BlockingStream:
    def read(self, size):
        threading.Event().wait()


def rejected(action):
    try:
        action()
    except RuntimeError:
        return True
    return False


def main():
    oversized = io.BytesIO(struct.pack(">I", run_scenario.MAX_FRAME_BYTES + 1))
    if not rejected(lambda: run_scenario.read_message(oversized, sentinel_pb2.Envelope)):
        raise RuntimeError("runner accepted an oversized frame")
    timeout = run_scenario.RESPONSE_TIMEOUT_SECONDS
    run_scenario.RESPONSE_TIMEOUT_SECONDS = 0.01
    try:
        if not rejected(lambda: run_scenario.read_message(BlockingStream(), sentinel_pb2.Envelope)):
            raise RuntimeError("runner accepted an unresponsive process")
    finally:
        run_scenario.RESPONSE_TIMEOUT_SECONDS = timeout
    process = subprocess.Popen(
        [sys.executable, "-c", "raise SystemExit(3)"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if not run_scenario.stop_process(process):
        raise RuntimeError("runner lost a child process failure")
    print("runner tests passed")


if __name__ == "__main__":
    main()
