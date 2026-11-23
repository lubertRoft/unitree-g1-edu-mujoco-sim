"""Unit tests for arm_sdk blending / stream loss (no DDS)."""
import pytest
from types import SimpleNamespace

from controller import config
from controller.blend import ArmBlender, ArmCommand, from_msg

Q0 = config.DEFAULT_Q[12:]


def cmd(w, q=1.0, kp=60.0, kd=1.5, dq=0.2, tau=0.3):
    n = 17
    return ArmCommand(w=w, q=[q] * n, dq=[dq] * n, tau=[tau] * n, kp=[kp] * n, kd=[kd] * n)


def run(b, c, rx=0.0, now=0.0, ready=True):
    return b.step(c, rx, now, ready)


def test_zero_weight_holds_default():
    w, q, dq, tau, kp, kd = run(ArmBlender(), cmd(0.0))
    assert w == 0 and q == Q0 and dq == [0] * 17 and tau == [0] * 17
    assert kp == config.DEFAULT_KP[12:] and kd == config.DEFAULT_KD[12:]


def test_full_weight_tracks_client():
    w, q, dq, tau, kp, kd = run(ArmBlender(), cmd(1.0))
    assert w == 1 and q == [1.0] * 17 and dq == [0.2] * 17 and tau == [0.3] * 17
    assert kp == [60.0] * 17 and kd == [1.5] * 17


def test_half_weight_is_midpoint_and_scales_dq_tau():
    w, q, dq, tau, kp, kd = run(ArmBlender(), cmd(0.5))
    assert w == 0.5
    assert q == pytest.approx([0.5 * 1.0 + 0.5 * d for d in Q0])
    assert dq == pytest.approx([0.1] * 17) and tau == pytest.approx([0.15] * 17)
    assert kp == [60.0] * 17           # client gains once w > 0


@pytest.mark.parametrize("w_in,expected", [(1.7, 1.0), (-0.3, 0.0), (1.0, 1.0), (0.0, 0.0)])
def test_out_of_range_weight_is_clamped(w_in, expected):
    w, q, *_ = run(ArmBlender(), cmd(w_in))
    assert w == expected
    assert q == ([1.0] * 17 if expected == 1.0 else Q0)


def test_ignored_before_ready():
    w, q, *_ = run(ArmBlender(), cmd(1.0), ready=False)
    assert w == 0 and q == Q0


def test_no_message_means_default():
    w, q, *_ = run(ArmBlender(), None, rx=None)
    assert w == 0 and q == Q0


def test_leg_commands_have_no_effect():
    mc = [SimpleNamespace(q=9.0, dq=9.0, tau=9.0, kp=9.0, kd=9.0) for _ in range(35)]
    for i in range(12, 29):
        mc[i] = SimpleNamespace(q=0.4, dq=0.0, tau=0.0, kp=50.0, kd=1.0)
    mc[29] = SimpleNamespace(q=1.0, dq=0, tau=0, kp=0, kd=0)
    c = from_msg(SimpleNamespace(motor_cmd=mc))
    assert len(c.q) == 17 and c.w == 1.0 and all(v == 0.4 for v in c.q)   # legs (0-11) never read


def test_stream_loss_ramps_to_default():
    b = ArmBlender()
    c, rx = cmd(1.0), 0.0
    dt, t = 0.002, 0.0
    # fresh for 0.4 s: still full weight
    while t < 0.4:
        w, *_ = b.step(c, rx, t, True); t += dt
    assert w == 1.0
    # stale after 0.5 s: w ramps 1 -> 0 over 1 s
    ws = []
    while t < 2.0:
        w, q, *_ = b.step(c, rx, t, True); ws.append((t, w)); t += dt
    t_ramp_start = next(tt for tt, ww in ws if ww < 1.0)
    assert 0.49 < t_ramp_start < 0.55
    mid = next(ww for tt, ww in ws if tt >= 1.0)
    assert 0.4 < mid < 0.6                              # halfway through the ramp
    assert next(ww for tt, ww in ws if tt >= 1.55) == 0.0   # default reached by 0.5 + 1.0 s
    assert q == Q0
    assert all(a >= b for (_, a), (_, b) in zip(ws, ws[1:]))  # monotone: smooth


def test_stream_resume_restores_weight():
    b = ArmBlender()
    t = 0.0
    while t < 1.0:
        b.step(cmd(1.0), 0.0, t, True); t += 0.002
    w, *_ = b.step(cmd(1.0), 1.0, t, True)    # fresh message arrives
    assert w == 1.0


def test_uncommanded_joint_keeps_default_pose_and_gains():
    c = cmd(1.0)
    c.kp[0] = c.kd[0] = c.tau[0] = 0.0          # e.g. waist yaw not commanded by an arm-only client
    w, q, dq, tau, kp, kd = run(ArmBlender(), c)
    assert q[0] == Q0[0] and kp[0] == config.DEFAULT_KP[12] and kd[0] == config.DEFAULT_KD[12]
    assert q[3] == 1.0 and kp[3] == 60.0         # commanded joints unaffected
