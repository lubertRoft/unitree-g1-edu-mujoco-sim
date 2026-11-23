"""unitree_rl_gym G1 `motion.pt` policy (observation layout: controller/policy/README.md)."""
import math
import os

import numpy as np
import torch

from .. import config
from . import Policy, RobotState

POLICY_PATH = os.path.join(os.environ.get("RL_GYM_DIR", "/opt/unitree_rl_gym"),
                           "deploy/pre_train/g1/motion.pt")


def projected_gravity(quat):
    w, x, y, z = quat
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


class RlGymG1Policy(Policy):
    ANG_VEL_SCALE, DOF_POS_SCALE, DOF_VEL_SCALE = 0.25, 1.0, 0.05
    ACTION_SCALE, PERIOD = 0.25, 0.8

    def __init__(self, path=POLICY_PATH):
        torch.set_num_threads(1)
        self.net = torch.jit.load(path, map_location="cpu")
        self.default_q = np.array(config.LEG_DEFAULT, dtype=np.float32)
        self.kp = np.array(config.LEG_KP, dtype=np.float32)
        self.kd = np.array(config.LEG_KD, dtype=np.float32)
        self.cmd = np.zeros(3, dtype=np.float32)      # zero velocity command: stand
        self.action = np.zeros(12, dtype=np.float32)
        self._arm_forward = 0.0

    def reset(self, state):
        self.action[:] = 0.0
        self._arm_forward = 0.0

    def observation(self, s: RobotState):
        obs = np.zeros(47, dtype=np.float32)
        ph = (s.time % self.PERIOD) / self.PERIOD
        obs[0:3] = s.gyro * self.ANG_VEL_SCALE
        obs[3:6] = projected_gravity(s.quat)
        obs[6:9] = self.cmd * np.array([2.0, 2.0, 0.25], dtype=np.float32)
        obs[9:21] = (s.q[:12] - self.default_q) * self.DOF_POS_SCALE
        obs[21:33] = s.dq[:12] * self.DOF_VEL_SCALE
        obs[33:45] = self.action
        obs[45:47] = (math.sin(2 * math.pi * ph), math.cos(2 * math.pi * ph))
        return obs

    def step(self, s):
        with torch.no_grad():
            self.action = self.net(torch.from_numpy(self.observation(s)).unsqueeze(0)).numpy()[0]
        target = self.action * self.ACTION_SCALE + self.default_q
        forward = -0.5 * (s.q[15] + s.q[22]) / config.HIP_BIAS_FULL_AT       # 0 arms down .. 1 arms forward
        forward = min(max(float(forward), 0.0), 1.0)
        alpha = 0.02 / (config.HIP_BIAS_FILTER + 0.02)                        # policy runs at 50 Hz
        self._arm_forward += alpha * (forward - self._arm_forward)
        target[[0, 6]] += config.HIP_PITCH_BIAS * self._arm_forward
        return target
