"""DDS plumbing: channel init, lowstate subscriber, lowcmd publisher with CRC, arm_sdk subscriber."""
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # noqa: F401 (re-exported)
from unitree_sdk2py.utils.crc import CRC

from . import config

_initialized = False


def init():
    global _initialized
    if not _initialized:
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        _initialized = True


def wait_for_lowstate(timeout):
    """True if an rt/lowstate sample arrives within `timeout` s (used by the supervisor)."""
    init()
    sub = ChannelSubscriber(config.TOPIC_LOWSTATE, LowState_)
    sub.Init()
    return sub.Read(timeout) is not None


class Latest:
    """Keeps the latest message of a topic and its arrival time."""

    def __init__(self, topic, typ):
        self.msg = None
        self.stamp = None
        self._lock = threading.Condition()
        self._sub = ChannelSubscriber(topic, typ)
        self._sub.Init(self._on, 10)

    def _on(self, msg):
        with self._lock:
            self.msg, self.stamp = msg, time.monotonic()
            self._lock.notify_all()

    def wait_new(self, last_stamp, timeout):
        """Block until a message newer than `last_stamp` arrives (or timeout); returns (msg, stamp)."""
        with self._lock:
            self._lock.wait_for(lambda: self.stamp is not None and self.stamp != last_stamp, timeout)
            return self.msg, self.stamp

    def get(self):
        with self._lock:
            return self.msg, self.stamp


class LowCmdPublisher:
    def __init__(self):
        self._pub = ChannelPublisher(config.TOPIC_LOWCMD, LowCmd_)
        self._pub.Init()
        self._crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()

    def send(self, mode_machine, q, dq, kp, kd, tau, echo_tick=0):
        """echo_tick: tick (ms) of the lowstate this command answers; lets the sim run in lockstep."""
        m = self.msg
        m.reserve[0] = int(echo_tick)
        m.mode_pr = 0
        m.mode_machine = mode_machine
        for i in range(config.NUM_MOTORS):
            c = m.motor_cmd[i]
            c.mode = 1
            c.q, c.dq, c.kp, c.kd, c.tau = float(q[i]), float(dq[i]), float(kp[i]), float(kd[i]), float(tau[i])
        m.crc = self._crc.Crc(m)
        self._pub.Write(m)
