"""Run the real controller classes (Machine + RlGymG1Policy) against the sim model in lockstep, no DDS.
Diagnostic: separates controller-logic effects from DDS/timing effects.  Usage: inproc_stack.py [seconds] [obs_from_sensors]"""
import math, sys, logging
import mujoco, numpy as np
sys.path.insert(0, "/app/sim"); sys.path.insert(0, "/app")
import config as simcfg
from band import Band
from controller import config
from controller.machine import Machine, S
from controller.policy import RobotState
from controller.policy.rl_gym import RlGymG1Policy

logging.basicConfig(level=logging.INFO, format="%(message)s")
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 60
DELAY = int(sys.argv[2]) if len(sys.argv) > 2 else 0          # command applied this many physics steps late
JITTER = int(sys.argv[3]) if len(sys.argv) > 3 else 0         # plus random 0..JITTER extra steps
from collections import deque
rng = np.random.default_rng(1)
queue = deque()
m = mujoco.MjModel.from_xml_path(simcfg.ROBOT_SCENE); d = mujoco.MjData(m); m.opt.timestep = 0.002
mujoco.mj_resetData(m, d); d.qpos[2] = simcfg.SPAWN_PELVIS_Z; d.qpos[7:19] = simcfg.SPAWN_LEG_ANGLES; mujoco.mj_forward(m, d)
torso = m.body("torso_link").id
band = Band(simcfg, d.xpos[torso][:2], d.xpos[torso][2])
n = 29

class B:  # Band API used by Machine
    status = lambda self: (band.enabled, band.height)
    def set_height(self, z): band.handle(f"height {z}")
    def release(self): band.handle("release")

mach = Machine(RlGymG1Policy(), B())
def state():
    sd = d.sensordata          # same (one-step-stale) sensor data the sim publishes
    o = 3 * n
    return RobotState(time=d.time, quat=sd[o:o+4].copy(), gyro=sd[o+4:o+7].astype(np.float32),
                      q=sd[:n].astype(np.float32), dq=sd[n:2*n].astype(np.float32))
cmd = None; path = 0.0; last = None; t_ready = None
for k in range(int(SECONDS / 0.002)):
    if k % 1 == 0:
        cmd = mach.step(state(), d.time)
    queue.append((k + DELAY + (int(rng.integers(0, JITTER + 1)) if JITTER else 0), cmd))
    due = [c for c in queue if c[0] <= k]
    if due:
        applied = max(due, key=lambda c: c[0])[1]
        for c in due: queue.remove(c)
        active = applied
    elif k == 0:
        active = cmd
    cmd_a = active
    sd = d.sensordata
    d.ctrl[:n] = cmd_a.tau + cmd_a.kp * (cmd_a.q - sd[:n]) + cmd_a.kd * (cmd_a.dq - sd[n:2*n])
    d.xfrc_applied[torso, :3] = band.force(d.xpos[torso], d.cvel[torso][3:])
    mujoco.mj_step(m, d)
    if mach.state == S.READY and t_ready is None: t_ready = d.time; last = d.qpos[:2].copy()
    if t_ready is not None and k % 50 == 0:
        path += float(np.linalg.norm(d.qpos[:2] - last)); last = d.qpos[:2].copy()
    if mach.state == S.FALLEN: print("FALLEN at", d.time); break
T = d.time - (t_ready or 0)
print(f"RESULT state={mach.state.value} READY at {t_ready:.1f}s; path {path:.1f} m in {T:.0f}s ({path/max(T,1):.2f} m/s)")
