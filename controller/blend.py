"""arm_sdk blending and stream-loss logic. Pure Python: no DDS, unit-testable."""
from dataclasses import dataclass

from . import config

N_WA = 17  # joints 12..28


def clamp01(x):
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


@dataclass
class ArmCommand:
    """Plain-data view of an arm_sdk LowCmd_ (waist+arms 12..28 plus weight)."""
    w: float
    q: list
    dq: list
    tau: list
    kp: list
    kd: list


def from_msg(msg):
    mc = msg.motor_cmd
    idx = range(12, 29)
    return ArmCommand(
        w=float(mc[config.ARM_SDK_WEIGHT_IDX].q),
        q=[mc[i].q for i in idx], dq=[mc[i].dq for i in idx], tau=[mc[i].tau for i in idx],
        kp=[mc[i].kp for i in idx], kd=[mc[i].kd for i in idx],
    )


class ArmBlender:
    """Computes the effective weight (with stream-loss ramp) and blended waist/arm targets.

    `step(cmd, rx_time, now, ready)` -> (w_eff, q, dq, tau, kp, kd) for joints 12..28.
    Leg fields of arm_sdk messages are never read (see from_msg).
    A joint whose kp, kd and tau are all 0 is treated as uncommanded and held at the default pose.
    """

    def __init__(self, default_q=None, default_kp=None, default_kd=None,
                 timeout=config.ARM_SDK_TIMEOUT, ramp=config.ARM_SDK_RAMP):
        self.q0 = list(default_q if default_q is not None else config.DEFAULT_Q[12:])
        self.kp0 = list(default_kp if default_kp is not None else config.DEFAULT_KP[12:])
        self.kd0 = list(default_kd if default_kd is not None else config.DEFAULT_KD[12:])
        self.timeout, self.ramp = timeout, ramp
        self._scale = 1.0     # stream-loss multiplier on w, 1 -> 0 over `ramp` s
        self._last_t = None

    def _default(self):
        return 0.0, list(self.q0), [0.0] * N_WA, [0.0] * N_WA, list(self.kp0), list(self.kd0)

    def step(self, cmd, rx_time, now, ready):
        dt = 0.0 if self._last_t is None else max(now - self._last_t, 0.0)
        self._last_t = now
        w_in = clamp01(cmd.w) if (ready and cmd is not None) else 0.0
        if w_in <= 0.0:
            self._scale = 1.0
            return self._default()
        if now - rx_time > self.timeout:
            self._scale = max(self._scale - dt / self.ramp, 0.0)
        else:
            self._scale = 1.0
        w = w_in * self._scale
        if w <= 0.0:
            return self._default()
        q, dq, tau, kp, kd = [], [], [], [], []
        for i in range(N_WA):
            if cmd.kp[i] == 0.0 and cmd.kd[i] == 0.0 and cmd.tau[i] == 0.0:
                # Uncommanded joint (e.g. waist for arm-only clients): keep holding the default pose,
                # as the real onboard controller does.
                q.append(self.q0[i]); dq.append(0.0); tau.append(0.0)
                kp.append(self.kp0[i]); kd.append(self.kd0[i])
            else:
                q.append(w * cmd.q[i] + (1.0 - w) * self.q0[i])
                dq.append(w * cmd.dq[i]); tau.append(w * cmd.tau[i])
                kp.append(cmd.kp[i]); kd.append(cmd.kd[i])
        return w, q, dq, tau, kp, kd
