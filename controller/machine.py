"""Startup state machine + command assembly. No DDS: takes RobotState, returns joint commands."""
import logging
import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from . import config
from .blend import ArmBlender
from .policy import Policy, RobotState

log = logging.getLogger("controller")


class S(str, Enum):
    BOOT = "BOOT"
    DAMP = "DAMP"
    STAND = "STAND"
    BALANCE = "BALANCE"
    READY = "READY"
    FALLEN = "FALLEN"


@dataclass
class Command:
    q: np.ndarray
    dq: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau: np.ndarray


def roll_pitch(quat):
    w, x, y, z = quat
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


class BandPort:
    """What the machine needs from the elastic band (real impl: sim.band.BandClient)."""
    def status(self): ...
    def set_height(self, z): ...
    def release(self): ...


class Machine:
    def __init__(self, policy: Policy, band, blender=None, fall_threshold=config.FALL_ROLL_PITCH):
        self.policy, self.band = policy, band
        self.blender = blender or ArmBlender()
        self.fall_threshold = fall_threshold
        self.state = S.BOOT
        self._t_enter = None
        self._q_start = None
        self._leg_target = None
        self._tick = 0
        self._policy_bucket = None   # sim-time slot (20 ms) of the last policy update
        self._band_h0 = self._band_h = None
        self.arm_w = 0.0

    # -- helpers --------------------------------------------------------
    def _goto(self, new, now):
        if new != self.state:
            log.info("state %s -> %s", self.state.value, new.value)
        self.state, self._t_enter = new, now

    def _damp(self):
        z = np.zeros(config.NUM_MOTORS)
        return Command(q=z.copy(), dq=z.copy(), kp=z.copy(), kd=np.full(config.NUM_MOTORS, config.DAMP_KD), tau=z.copy())

    def _hold_default_wa(self, cmd):
        cmd.q[12:] = config.DEFAULT_Q[12:]
        cmd.kp[12:] = config.DEFAULT_KP[12:]
        cmd.kd[12:] = config.DEFAULT_KD[12:]

    # -- main entry -----------------------------------------------------
    def step(self, s: RobotState, now, arm_cmd=None, arm_rx=None):
        """now: monotonic seconds. Returns Command for all 29 joints."""
        if self.state == S.BOOT:
            self._goto(S.DAMP, now)

        # fall protection in every state that is actively balancing
        if self.state in (S.STAND, S.BALANCE, S.READY):
            r, p = roll_pitch(s.quat)
            if max(abs(r), abs(p)) > self.fall_threshold:
                log.error("FALL detected: roll=%.2f pitch=%.2f rad (threshold %.2f); damping all joints",
                          r, p, self.fall_threshold)
                self._goto(S.FALLEN, now)

        el = now - self._t_enter
        if self.state == S.DAMP:
            cmd = self._damp()
            if el >= config.DAMP_TIME:
                self._q_start = s.q.copy()
                self._band_h0 = self._band_h = self.band.status()[1]
                self._goto(S.STAND, now)
            return cmd

        if self.state == S.FALLEN:
            return self._damp()

        cmd = Command(q=np.zeros(29), dq=np.zeros(29), kp=np.zeros(29), kd=np.zeros(29), tau=np.zeros(29))
        default = np.array(config.DEFAULT_Q)

        if self.state == S.STAND:
            a = min(el / config.STAND_RAMP, 1.0)
            a = 0.5 - 0.5 * math.cos(math.pi * a)   # smooth ramp
            cmd.q[:] = (1 - a) * self._q_start + a * default
            cmd.kp[:], cmd.kd[:] = config.DEFAULT_KP, config.DEFAULT_KD
            if el >= config.STAND_RAMP:
                # lower the band at a fixed speed until the drop is reached
                target = self._band_h0 - config.BAND_LOWER_DROP
                if self._band_h > target:
                    self._band_h = max(self._band_h - config.BAND_LOWER_SPEED / config.CONTROL_HZ, target)
                    self.band.set_height(self._band_h) if self._tick % 10 == 0 else None
                else:
                    self.band.set_height(target)
                    self._goto(S.BALANCE, now)
                    self.policy.reset(s)
                    self._leg_target = default[:12].copy()
                    self._policy_bucket = None
            self._tick += 1
            return cmd

        # BALANCE / READY: legs from the policy at 50 Hz, waist/arms PD or arm_sdk blend
        # Step the policy on the *sim* clock (rt/lowstate tick), once per 20 ms slot, so the update interval seen by
        # the policy is exactly 20 ms as in training (wall-clock counting jitters between 14 and 26 ms of sim time).
        bucket = int(round(s.time * 1000.0)) // config.POLICY_PERIOD_MS
        if bucket != self._policy_bucket:
            self._policy_bucket = bucket
            self._leg_target = self.policy.step(s)
        self._tick += 1
        cmd.q[:12], cmd.kp[:12], cmd.kd[:12] = self._leg_target, self.policy.kp, self.policy.kd
        self._hold_default_wa(cmd)

        if self.state == S.BALANCE and el >= config.BALANCE_HOLD:
            self.band.release()
            self._goto(S.READY, now)
        if self.state == S.READY:
            w, q, dq, tau, kp, kd = self.blender.step(arm_cmd, arm_rx, now, ready=True)
            self.arm_w = w
            cmd.q[12:], cmd.dq[12:], cmd.tau[12:], cmd.kp[12:], cmd.kd[12:] = q, dq, tau, kp, kd
        return cmd
