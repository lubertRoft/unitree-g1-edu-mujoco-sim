"""Balance policy plug-in interface.

A policy maps the robot state to *leg* joint targets (indices 0-11) at 50 Hz.
To swap in another policy, implement `Policy` and select it in `controller/main.py`
(see README, "Swapping the policy").
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class RobotState:
    """What a policy may observe (all from rt/lowstate)."""
    time: float                 # sim time, s (lowstate tick / 1000)
    quat: np.ndarray            # pelvis IMU quaternion (w, x, y, z)
    gyro: np.ndarray            # pelvis IMU angular velocity, body frame
    q: np.ndarray               # 29 joint positions
    dq: np.ndarray              # 29 joint velocities


class Policy(ABC):
    #: leg PD gains/default pose used while this policy is active
    kp: np.ndarray
    kd: np.ndarray
    default_q: np.ndarray

    @abstractmethod
    def reset(self, state: RobotState) -> None:
        """Called when the policy is activated."""

    @abstractmethod
    def step(self, state: RobotState) -> np.ndarray:
        """Return 12 leg joint targets for indices 0-11. Called at 50 Hz."""
