# rl_gym G1 policy: recorded interface (task 4.1)

Source: `unitree_rl_gym` @ 276801e (pinned), `deploy/deploy_mujoco/deploy_mujoco.py`,
`deploy/deploy_mujoco/configs/g1.yaml`, weights `deploy/pre_train/g1/motion.pt` (TorchScript MLP).
The policy is the **12-DoF legs-only** G1 (waist/arms fixed in training).

| Item | Value |
|---|---|
| Policy rate | 50 Hz (`control_decimation` 10 at `simulation_dt` 0.002) |
| Observation (47) | `[0:3]` base angular velocity (body frame) x 0.25 · `[3:6]` projected gravity (from pelvis quaternion w,x,y,z) · `[6:9]` command (vx, vy, yaw) x `[2.0, 2.0, 0.25]` · `[9:21]` (q - default) x 1.0 · `[21:33]` dq x 0.05 · `[33:45]` last action · `[45:47]` `sin, cos` of gait phase |
| Gait phase | `phase = (t % 0.8) / 0.8`, `t` = sim time; period **0.8 s** |
| Action | 12 values; target `q = action * 0.25 + default` |
| Default leg angles | `[-0.1, 0, 0, 0.3, -0.2, 0]` per leg (hip p/r/y, knee, ankle p/r) |
| Leg PD gains | kp `[100,100,100,150,40,40]`, kd `[2,2,2,4,2,2]` per leg; target dq = 0 |
| Joint order | hardware indices 0-5 left leg, 6-11 right leg (same as DDS) |
| Upstream command | `cmd_init [0.5, 0, 0]`; this project uses **zero** (stand) |

Not controlled by the policy (held by our PD / arm_sdk blend): waist 12-14, arms 15-28.

## Spike results (tasks 4.3-4.5)

In-process MuJoCo on the 29-DoF scene (`scripts/spike_policy.py`), zero velocity command, arms/waist PD-held
at the default pose (arms down), legs from the policy at 50 Hz:

| Scenario | Result |
|---|---|
| Band-held stand, lower 0.12 m, policy on, release | Upright for 71 s free standing; max tilt 0.13 rad, z ~0.77 m |
| Arm swing +-0.6 rad @ 0.3 Hz (both arms) | Upright 41 s; max tilt 0.28 rad |
| Arm swing +-1.0 rad @ 0.5 Hz | Upright 41 s; max tilt 0.31 rad |
| Arm swing +-1.5 rad @ 1.0 Hz | **Falls** after 21 s (tilt 0.77 rad) |

Observations: the gait-phase policy **marches in place** and the base drifts (about 0.05 m/s, e.g. 3.9 m in
71 s, along a slow arc). It stays balanced, so this is accepted for v1 (documented fidelity limit).
Fall threshold: tilt reached 0.31 rad in moderate motion and 0.77 rad when falling, so the controller uses 0.6 rad.

**Decision:** proceed with the `rl_gym` policy unchanged (default pose, gains and scales as above).

### Arm-pose robustness and the scheduled hip-pitch bias (found while writing the E2E tests)

Holding both arms forward (shoulder pitch -0.8, shoulder roll +-0.5, elbow 0.7; `examples/arm_sdk_example.py`)
tipped the unmodified policy over (>=70 % of that pose fails; 50 % is fine): the arms shift the centre of mass
forward and the policy was trained with fixed arms. A feed-forward bias on both hip-pitch targets fixes the held pose
in the standalone spike: 0.05 rad still fails, 0.10 / 0.15 / 0.20 rad stay up.

A **constant** bias is not acceptable, though: in the real DDS stack it made the *idle* robot fall about 3 minutes
after READY (the 10-minute standalone spike did not show this, so spike results are indicative only), while the
stack without bias idled 5 minutes fine. The controller therefore schedules the bias on the arm pose:
`bias = HIP_PITCH_BIAS (0.15 rad) * clip(-mean(q15, q22) / 0.8, 0, 1)`, low-passed with a 0.5 s time constant and
added to the policy *output* only (the observation is unchanged). With the arms down it is zero, so idle behavior
equals the unmodified policy.

### Latency sensitivity (found while investigating base wandering in the viewer)

The standalone spike and the controller classes run in lockstep with no DDS wander at about 0.11-0.14 m/s. Over DDS
the robot initially walked at 0.3-0.45 m/s. In-process experiments (`scripts/inproc_stack.py <s> <delay> <jitter>`)
show why: delaying the command by 1 physics step gives 0.36 m/s, 2 steps 0.47 m/s, and delay with jitter can topple the
robot. The sim/controller lockstep handshake (README, "Controller / simulator lockstep") removes the delay and brings
the live stack back to 0.14 m/s. Observation-only delay up to 3 steps and a stale-state PD loop did *not* matter.
