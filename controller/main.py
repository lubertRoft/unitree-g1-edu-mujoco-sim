"""Emulated G1 onboard controller: rt/arm_sdk + rt/lowstate in, rt/lowcmd out (500 Hz)."""
import logging
import sys
import time

import numpy as np

from . import config, dds
from .blend import from_msg
from .machine import Machine
from .policy import RobotState
from .policy.rl_gym import RlGymG1Policy


def to_state(ls):
    return RobotState(
        time=ls.tick / 1000.0,
        quat=np.array(ls.imu_state.quaternion, dtype=np.float64),
        gyro=np.array(ls.imu_state.gyroscope, dtype=np.float32),
        q=np.array([ls.motor_state[i].q for i in range(config.NUM_MOTORS)], dtype=np.float32),
        dq=np.array([ls.motor_state[i].dq for i in range(config.NUM_MOTORS)], dtype=np.float32),
    )


def main():
    logging.basicConfig(level=logging.INFO, stream=sys.stdout,
                        format="%(asctime)s [controller] %(levelname)s %(message)s")
    log = logging.getLogger("controller")
    sys.path.insert(0, "/app/sim")
    from band import BandClient

    dds.init()
    lowstate = dds.Latest(config.TOPIC_LOWSTATE, dds.LowState_)
    arm_sdk = dds.Latest(config.TOPIC_ARM_SDK, dds.LowCmd_)
    pub = dds.LowCmdPublisher()
    machine = Machine(RlGymG1Policy(), BandClient(config.BAND_SOCKET))

    log.info("waiting for rt/lowstate (domain %d, %s)", config.DOMAIN_ID, config.INTERFACE)
    while lowstate.get()[0] is None:
        time.sleep(0.05)
    log.info("state BOOT")

    # Lockstep with the simulator: one control cycle per rt/lowstate sample (500 Hz of sim time), like the real
    # robot's fixed-rate cycle. No free-running timer, so there is no phase jitter against physics.
    stamp = None
    warned = False
    lat, lat_policy = [], []
    while True:
        ls, new_stamp = lowstate.wait_new(stamp, timeout=1.0)
        if new_stamp == stamp:
            if not warned:
                log.warning("no rt/lowstate for 1 s; waiting")
                warned = True
            continue
        warned, stamp = False, new_stamp
        arm_msg, arm_rx = arm_sdk.get()
        arm_cmd = from_msg(arm_msg) if arm_msg is not None else None
        cmd = machine.step(to_state(ls), time.monotonic(), arm_cmd, arm_rx)
        pub.send(ls.mode_machine, cmd.q, cmd.dq, cmd.kp, cmd.kd, cmd.tau, echo_tick=ls.tick)
        # latency from rt/lowstate arrival to rt/lowcmd written (the sim waits for this in lockstep mode)
        lat.append((time.monotonic() - new_stamp) * 1000.0)
        if len(lat) >= 5000:
            a = np.array(lat)
            log.info("cycle latency ms: mean %.2f p50 %.2f p99 %.2f max %.2f", a.mean(), np.percentile(a, 50),
                     np.percentile(a, 99), a.max())
            lat.clear()


if __name__ == "__main__":
    main()
