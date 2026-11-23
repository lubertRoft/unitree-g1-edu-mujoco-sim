"""G1 29-DoF MuJoCo simulator entry point.

Wraps the stock ``unitree_mujoco`` DDS bridge (rt/lowcmd in, rt/lowstate out)
and adds: headless mode, PD torque evaluated every physics step, CRC and
``mode_machine`` on rt/lowstate, drift-free real-time pacing and a
programmatic elastic band.
"""
import signal
import threading
import time

import mujoco
import numpy as np

import config
from band import Band

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py_bridge import UnitreeSdk2Bridge  # upstream, pinned


class _CrcWriter:
    """Stamp tick / mode_machine / crc on every rt/lowstate sample."""

    def __init__(self, pub, data):
        self._pub, self._data, self._crc = pub, data, CRC()

    def Write(self, msg, *a, **kw):
        msg.tick = int(self._data.time * 1000)
        msg.mode_machine = config.MODE_MACHINE
        msg.crc = self._crc.Crc(msg)
        return self._pub.Write(msg, *a, **kw)

    def __getattr__(self, name):
        return getattr(self._pub, name)


class G1Bridge(UnitreeSdk2Bridge):
    """Stock bridge + lockstep with the onboard controller.

    The balance policy was trained without actuation latency; two processes talking over DDS add >= 1 physics step
    (2 ms) of delay, which makes it walk instead of stand. If the controller echoes the tick of the state it acted
    on in ``LowCmd_.reserve[0]``, the sim waits (up to SYNC_TIMEOUT) for that answer before the next step, like an
    onboard controller synchronised with the motor drive. Raw rt/lowcmd clients (no echo) are never waited for.
    """
    SYNC_TIMEOUT = 0.0018
    SYNC_DISABLE_AFTER = 50      # consecutive misses before the wait is suspended (controller gone)

    def __init__(self, model, data):
        self._cmd = None
        self._cmd_tick = 0
        self._cv = threading.Condition()
        self.sync_active = False
        self.sync_misses = self.sync_total = self._miss_run = 0
        super().__init__(model, data)
        self.low_state_puber = _CrcWriter(self.low_state_puber, data)

    def PublishLowState(self):
        # Upstream calls this from a free-running timer thread, asynchronously to physics: samples then repeat or
        # skip sim ticks and can be torn (mj_step releases the GIL). Ignore the timer; publish_state() is called
        # from the physics loop right after every step instead (exactly 500 Hz of sim time, consistent data).
        pass

    def publish_state(self):
        super().PublishLowState()
        if not self.sync_active:
            return
        tick = int(self.mj_data.time * 1000)
        end = time.perf_counter() + self.SYNC_TIMEOUT
        with self._cv:
            while self._cmd_tick != tick:
                left = end - time.perf_counter()
                if left <= 0:
                    break
                self._cv.wait(left)
            hit = self._cmd_tick == tick
        self.sync_total += 1
        if hit:
            self._miss_run = 0
        else:
            self.sync_misses += 1
            self._miss_run += 1
            if self._miss_run >= self.SYNC_DISABLE_AFTER:
                self.sync_active = False       # controller gone/stalled: stop waiting until it echoes again

    def LowCmdHandler(self, msg):
        n = self.num_motor
        mc = msg.motor_cmd
        cmd = np.array([(mc[i].q, mc[i].dq, mc[i].kp, mc[i].kd, mc[i].tau) for i in range(n)])
        echo = int(msg.reserve[0])
        with self._cv:
            self._cmd = cmd
            if echo:
                self._cmd_tick = echo
                self.sync_active = True
                self._miss_run = 0
            self._cv.notify_all()

    def apply_cmd(self):
        """tau = tau_ff + kp (q_des - q) + kd (dq_des - dq), each physics step."""
        cmd = self._cmd
        if cmd is None:
            return
        n = self.num_motor
        q = self.mj_data.sensordata[:n]
        dq = self.mj_data.sensordata[n:2 * n]
        self.mj_data.ctrl[:n] = cmd[:, 4] + cmd[:, 2] * (cmd[:, 0] - q) + cmd[:, 3] * (cmd[:, 1] - dq)


def spawn(model, data):
    mujoco.mj_resetData(model, data)
    data.qpos[2] = config.SPAWN_PELVIS_Z
    data.qpos[7:7 + 12] = config.SPAWN_LEG_ANGLES
    mujoco.mj_forward(model, data)


def main():
    model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
    data = mujoco.MjData(model)
    model.opt.timestep = config.SIMULATE_DT
    spawn(model, data)

    torso = model.body(config.BAND_ATTACH_BODY).id
    band = Band(config, data.xpos[torso][:2], data.xpos[torso][2])
    band.serve(config.BAND_SOCKET)

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    bridge = G1Bridge(model, data)

    stop = threading.Event()
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: stop.set())

    lock = threading.Lock()   # guards data for the viewer thread
    viewer = None
    if not config.HEADLESS:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)

        def view_loop():
            while viewer.is_running() and not stop.is_set():
                with lock:
                    viewer.sync()
                time.sleep(config.VIEWER_DT)
            stop.set()

        threading.Thread(target=view_loop, daemon=True, name="viewer").start()

    print(f"[sim] G1 29-DoF up: domain={config.DOMAIN_ID} iface={config.INTERFACE} "
          f"headless={config.HEADLESS} dt={model.opt.timestep}", flush=True)

    dt = model.opt.timestep
    deadline = time.perf_counter()
    while not stop.is_set():
        with lock:
            bridge.apply_cmd()
            data.xfrc_applied[torso, :3] = band.force(data.xpos[torso], data.cvel[torso][3:])
            mujoco.mj_step(model, data)
            bridge.publish_state()
        deadline += dt
        delay = deadline - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        elif delay < -0.1:  # fell far behind: resync instead of bursting
            deadline = time.perf_counter()
    if bridge.sync_total:
        print(f"[sim] controller sync: {bridge.sync_misses}/{bridge.sync_total} late answers", flush=True)
    print("[sim] stopped", flush=True)


if __name__ == "__main__":
    main()
