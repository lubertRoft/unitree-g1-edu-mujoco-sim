"""Arm-control example client over rt/arm_sdk (adapted from unitree_sdk2_python g1_arm7_sdk_dds_example).

Ramps the arm_sdk weight w 0 -> 1, moves both arms, moves back, ramps w 1 -> 0.

Simulation (this repo's container):   python3 arm_sdk_example.py            # domain 1, iface lo
Real robot:                           python3 arm_sdk_example.py --domain 0 --iface <eth iface>

Only publishes rt/arm_sdk. Never publish rt/lowcmd next to the onboard controller.
"""
import argparse
import sys
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

WEIGHT = 29                     # motor_cmd[29].q = arm_sdk weight
ARMS = list(range(15, 29))      # 15-21 left arm, 22-28 right arm
ARM_DOWN = [0.0, 0.25, 0.0, 0.5, 0.0, 0.0, 0.0,
            0.0, -0.25, 0.0, 0.5, 0.0, 0.0, 0.0]
# arms raised forward/sideways at moderate range
ARM_TARGET = [-0.8, 0.5, 0.0, 0.7, 0.0, 0.0, 0.0,
              -0.8, -0.5, 0.0, 0.7, 0.0, 0.0, 0.0]
KP, KD, DT = 60.0, 1.5, 0.02


def smooth(a):
    return 0.5 - 0.5 * np.cos(np.pi * np.clip(a, 0.0, 1.0))


class Client:
    def __init__(self):
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init()
        self.crc = CRC()
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.q = None

    def send(self, w, q):
        self.cmd.motor_cmd[WEIGHT].q = float(w)
        for j, qj in zip(ARMS, q):
            m = self.cmd.motor_cmd[j]
            m.q, m.dq, m.tau, m.kp, m.kd = float(qj), 0.0, 0.0, KP, KD
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def phase(self, name, seconds, fn):
        print(f"PHASE {name}", flush=True)
        t0 = time.time()
        while (t := time.time() - t0) < seconds:
            w, q = fn(t / seconds)
            self.send(w, q)
            time.sleep(DT)

    def run(self, ramp=2.0, move=4.0, hold=1.0):
        st = self.sub.Read(5.0)
        if st is None:
            sys.exit("no rt/lowstate: is the sim (or robot) running on this domain/interface?")
        q0 = np.array([st.motor_state[j].q for j in ARMS])
        tgt, down = np.array(ARM_TARGET), np.array(ARM_DOWN)
        self.phase("ramp_up", ramp, lambda a: (smooth(a), q0))                      # w 0 -> 1, hold current pose
        self.phase("move", move, lambda a: (1.0, q0 + smooth(a) * (tgt - q0)))      # arms to target
        self.phase("hold", hold, lambda a: (1.0, tgt))
        self.phase("return", move, lambda a: (1.0, tgt + smooth(a) * (down - tgt)))  # back to arms down
        self.phase("ramp_down", ramp, lambda a: (1.0 - smooth(a), down))            # w 1 -> 0
        self.send(0.0, down)
        print("DONE", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=1, help="DDS domain id (sim: 1, robot: 0)")
    ap.add_argument("--iface", default="lo", help="network interface (sim: lo, robot: eth iface)")
    ap.add_argument("--move-time", type=float, default=4.0)
    a = ap.parse_args()
    ChannelFactoryInitialize(a.domain, a.iface)
    Client().run(move=a.move_time)
