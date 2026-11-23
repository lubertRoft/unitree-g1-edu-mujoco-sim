"""State machine tests with a fake policy and band (no DDS)."""
import numpy as np

from controller import config
from controller.blend import ArmCommand
from controller.machine import Machine, S
from controller.policy import Policy, RobotState


class FakePolicy(Policy):
    kp = np.array(config.LEG_KP, dtype=float)
    kd = np.array(config.LEG_KD, dtype=float)
    default_q = np.array(config.LEG_DEFAULT, dtype=float)
    resets = 0

    def reset(self, s): self.resets += 1
    def step(self, s): return self.default_q + 0.01


class FakeBand:
    def __init__(self): self.h, self.enabled = 0.87, True
    def status(self): return self.enabled, self.h
    def set_height(self, z): self.h, self.enabled = z, True
    def release(self): self.enabled = False


def state(tilt=0.0):
    quat = np.array([np.cos(tilt / 2), np.sin(tilt / 2), 0, 0])
    return RobotState(time=0.0, quat=quat, gyro=np.zeros(3), q=np.zeros(29), dq=np.zeros(29))


def arm(w):
    n = 17
    return ArmCommand(w=w, q=[1.0] * n, dq=[0] * n, tau=[0] * n, kp=[60.0] * n, kd=[1.5] * n)


def advance(m, seconds, s=None, arm_cmd=None, t0=0.0, dt=0.002):
    t, cmd = t0, None
    while t < t0 + seconds:
        cmd = m.step(s or state(), t, arm_cmd, t if arm_cmd else None)
        t += dt
    return cmd, t


def test_startup_reaches_ready_in_order_and_releases_band():
    band, m = FakeBand(), Machine(FakePolicy(), FakeBand())
    m.band = band
    seen, t = [], 0.0
    while t < 30 and m.state != S.READY:
        m.step(state(), t); t += 0.002
        if not seen or seen[-1] != m.state: seen.append(m.state)
    assert seen == [S.DAMP, S.STAND, S.BALANCE, S.READY]
    assert t < 30 and band.enabled is False
    assert band.h == 0.87 - config.BAND_LOWER_DROP


def test_arm_commands_ignored_before_ready():
    m, t = Machine(FakePolicy(), FakeBand()), 0.0
    while m.state != S.READY:
        cmd = m.step(state(), t, arm(1.0), t)
        if m.state != S.READY:
            assert not np.any(np.isclose(cmd.q[12:], 1.0)), f"arm_sdk leaked in {m.state}"
            if m.state == S.BALANCE:                  # arms are held exactly at the default pose
                assert np.allclose(cmd.q[12:], config.DEFAULT_Q[12:])
            assert m.arm_w == 0.0
        t += 0.002
        assert t < 30


def test_ready_applies_arm_sdk_but_never_legs():
    m = Machine(FakePolicy(), FakeBand())
    _, t = advance(m, 12.0)
    assert m.state == S.READY
    cmd, _ = advance(m, 0.1, arm_cmd=arm(1.0), t0=t)
    assert np.allclose(cmd.q[12:], 1.0) and np.allclose(cmd.kp[12:], 60.0)
    assert np.allclose(cmd.q[:12], FakePolicy.default_q + 0.01)      # legs: policy only


def test_fall_protection_damps_everything_and_logs(caplog):
    m = Machine(FakePolicy(), FakeBand())
    _, t = advance(m, 12.0)
    assert m.state == S.READY
    with caplog.at_level("ERROR", logger="controller"):
        cmd, _ = advance(m, 0.01, s=state(tilt=0.8), t0=t)
    assert m.state == S.FALLEN
    assert np.all(cmd.kp == 0) and np.all(cmd.kd > 0) and np.all(cmd.tau == 0)
    assert any("FALL detected" in r.message for r in caplog.records)
    cmd, _ = advance(m, 0.1, s=state(), t0=t + 0.1, arm_cmd=arm(1.0))   # stays damped
    assert m.state == S.FALLEN and np.all(cmd.kp == 0)


def test_state_transitions_are_logged(caplog):
    m = Machine(FakePolicy(), FakeBand())
    with caplog.at_level("INFO", logger="controller"):
        advance(m, 12.0)
    text = " ".join(r.getMessage() for r in caplog.records)
    for name in ("DAMP", "STAND", "BALANCE", "READY"):
        assert f"-> {name}" in text


def test_policy_is_stepped_every_20ms_of_sim_time_despite_jittery_ticks():
    """Controller ticks jitter in wall time; the policy must still see exactly 20 ms of sim time between updates."""
    pol = FakePolicy()
    stamps = []
    orig = pol.step
    pol.step = lambda s: (stamps.append(s.time), orig(s))[1]
    m, rng, sim_t, wall = Machine(pol, FakeBand()), np.random.default_rng(0), 0.0, 0.0
    while m.state != S.READY:
        st = state(); st.time = sim_t
        m.step(st, wall)
        wall += 0.002
        sim_t += 0.002 * rng.choice([0, 1, 1, 1, 2])      # controller sometimes re-reads a sample, sometimes skips one
    for _ in range(3000):
        st = state(); st.time = sim_t
        m.step(st, wall); wall += 0.002
        sim_t += 0.002 * rng.choice([0, 1, 1, 1, 2])
    slots = [int(round(t * 1000)) // 20 for t in stamps]
    assert len(slots) == len(set(slots))                     # never twice in one 20 ms slot
    assert max(np.diff(slots)) <= 2                          # no slot skipped more than once
    assert len(stamps) > 100
