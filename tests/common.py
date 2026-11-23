"""Shared helpers for tests that talk to the running sim over DDS (domain 1, lo)."""
import math
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_


def init():
    ChannelFactoryInitialize(1, "lo")


def lowstate_sub():
    s = ChannelSubscriber("rt/lowstate", LowState_)
    s.Init()
    return s


def roll_pitch(ls):
    w, x, y, z = ls.imu_state.quaternion
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def max_tilt_over(sub, seconds):
    """Max |roll|,|pitch| seen over `seconds` of wall time."""
    end, worst = time.time() + seconds, 0.0
    while time.time() < end:
        ls = sub.Read(1.0)
        if ls is None:
            continue
        r, p = roll_pitch(ls)
        worst = max(worst, abs(r), abs(p))
    return worst
