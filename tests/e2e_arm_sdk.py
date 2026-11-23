"""E2E tests against a running, READY sim (tasks 8.2-8.5). Usage: e2e_arm_sdk.py <test>
tests: tracking | killed | idle | motion
"""
import signal
import subprocess
import sys
import time

sys.path.insert(0, "/app/tests")
import numpy as np
import common

ARM_DOWN = [0.0, 0.25, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, -0.25, 0.0, 0.5, 0.0, 0.0, 0.0]
ARM_TARGET = [-0.8, 0.5, 0.0, 0.7, 0.0, 0.0, 0.0, -0.8, -0.5, 0.0, 0.7, 0.0, 0.0, 0.0]
EX = ["python3", "-u", "/app/examples/arm_sdk_example.py"]


def arm_q(ls):
    return np.array([ls.motor_state[j].q for j in range(15, 29)])


def wait_ready(sub, tries=60):
    """READY heuristic: legs hold the default pose and the robot is upright."""
    for _ in range(tries):
        ls = sub.Read(1.0)
        if ls and abs(ls.motor_state[3].q - 0.3) < 0.15:
            return
        time.sleep(0.5)
    sys.exit("FAIL: robot not standing")


def start_example(extra=()):
    return subprocess.Popen(EX + list(extra), stdout=subprocess.PIPE, text=True, bufsize=1)


def phases(p):
    for line in p.stdout:
        if line.startswith("PHASE") or line.startswith("DONE"):
            yield line.split()[-1] if line.startswith("PHASE") else "DONE"


def report(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name, detail)
    sys.exit(0 if ok else 1)


def tracking(sub):
    p = start_example()
    held = back = None
    worst = 0.0
    for ph in phases(p):
        if ph == "hold":
            time.sleep(0.6)
            held = arm_q(sub.Read(1.0))
        if ph == "ramp_down":
            worst = max(worst, common.max_tilt_over(sub, 0.5))
        if ph == "DONE":
            time.sleep(3.0)   # let the gain switch at w=0 settle
            back = arm_q(sub.Read(1.0))
    err_hold = np.abs(held - np.array(ARM_TARGET)).max()
    err_back = np.abs(back - np.array(ARM_DOWN)).max()
    report("8.2 arms track at w=1 and return to arms-down at w=0",
           err_hold < 0.15 and err_back < 0.15, f"(hold err {err_hold:.3f} rad, back err {err_back:.3f} rad)")


def killed(sub):
    p = start_example()
    for ph in phases(p):
        if ph == "hold":
            break
    t_kill = time.time()
    p.send_signal(signal.SIGKILL)
    p.wait()
    down = np.array(ARM_DOWN)
    t_back = None
    while time.time() - t_kill < 4.0:
        ls = sub.Read(1.0)
        if ls and np.abs(arm_q(ls) - down).max() < 0.15:
            t_back = time.time() - t_kill
            break
    report("8.3 arms return to default within 1.5 s after client dies",
           t_back is not None and t_back <= 1.5, f"(back after {t_back} s)" if t_back else "(never)")


def idle(sub, seconds=300):
    worst = common.max_tilt_over(sub, seconds)
    ls = sub.Read(1.0)
    report("8.4 idle READY robot upright for 5 min", worst < 0.6, f"(max tilt {worst:.3f} rad over {seconds}s)")


def motion(sub):
    # moderate full-range arm motion: upright
    p = start_example(["--move-time", "3"])
    worst = 0.0
    while p.poll() is None:
        worst = max(worst, common.max_tilt_over(sub, 1.0))
    ok_moderate = worst < 0.6
    print(f"moderate: max tilt {worst:.3f} rad")
    return ok_moderate


def aggressive(sub, limit=90.0, amp=1.5, freq=1.0):
    """Swing the arms hard until the controller drops to damping (all kp == 0 on rt/lowcmd)."""
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    pub = ChannelPublisher("rt/arm_sdk", LowCmd_); pub.Init()
    lowcmd = ChannelSubscriber("rt/lowcmd", LowCmd_); lowcmd.Init()
    crc, msg = CRC(), unitree_hg_msg_dds__LowCmd_()
    down = np.array(ARM_DOWN)
    t0, damped = time.time(), False
    while time.time() - t0 < limit and not damped:
        t = time.time() - t0
        ph = 2 * np.pi * freq * t
        q = down.copy()
        for j in (0, 7):
            q[j] += amp * np.sin(ph)
        for j in (1, 8):
            q[j] += 0.5 * amp * np.sin(ph + 1.0) * (1 if j == 1 else -1)
        for j in (3, 10):
            q[j] += 0.5 * amp * np.sin(ph)
        msg.motor_cmd[29].q = float(min(t / 2.0, 1.0))
        for k, j in enumerate(range(15, 29)):
            m = msg.motor_cmd[j]; m.q, m.kp, m.kd = float(q[k]), 60.0, 1.5
        msg.crc = crc.Crc(msg); pub.Write(msg)
        c = lowcmd.Read(0.05)
        damped = c is not None and all(c.motor_cmd[i].kp == 0 and c.motor_cmd[i].kd > 0 for i in range(29))
        time.sleep(0.02)
    report("8.5b aggressive motion triggers damping on all joints", damped, f"(after {time.time() - t0:.1f} s)")


TESTS = {"aggressive": aggressive, "tracking": tracking, "killed": killed, "idle": idle, "motion": lambda s: report(
    "8.5a moderate arm motion keeps robot upright", motion(s))}

if __name__ == "__main__":
    common.init()
    sub = common.lowstate_sub()
    wait_ready(sub)
    TESTS[sys.argv[1]](sub)
