"""Container supervisor: start the sim, wait for rt/lowstate, start the controller.

Exits non-zero naming the failed process if either dies; SIGTERM stops both.
With arguments, runs them instead (e.g. `podman run ... g1-sim bash`).
"""
import os
import signal
import subprocess
import sys
import time

SIM = ["python3", "/app/sim/run_sim.py"]
CTRL = ["python3", "-m", "controller.main"]
LOWSTATE_TIMEOUT = 30.0


def wait_lowstate(sim, timeout):
    from controller.dds import wait_for_lowstate
    end = time.time() + timeout
    while time.time() < end:
        if sim.poll() is not None:
            return False
        if wait_for_lowstate(1.0):
            return True
    return False


def terminate(procs):
    for p in procs.values():
        if p.poll() is None:
            p.terminate()
    for p in procs.values():
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


def main():
    if len(sys.argv) > 1:
        os.execvp(sys.argv[1], sys.argv[1:])
    procs = {"sim": subprocess.Popen(SIM)}
    stopping = []
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: stopping.append(1))

    if not wait_lowstate(procs["sim"], LOWSTATE_TIMEOUT):
        print("[supervisor] FAILED: sim did not publish rt/lowstate", flush=True)
        terminate(procs)
        return 1
    procs["controller"] = subprocess.Popen(CTRL, env=dict(os.environ, PYTHONPATH="/app:" + os.environ.get("PYTHONPATH", "")))

    while not stopping:
        for name, p in procs.items():
            rc = p.poll()
            if rc is not None:
                print(f"[supervisor] FAILED: {name} exited unexpectedly (code {rc})", flush=True)
                terminate(procs)
                return rc if rc and rc > 0 else 1
        time.sleep(0.5)
    terminate(procs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
