# Unitree G1 EDU MuJoCo sim with `rt/arm_sdk`

A containerized MuJoCo simulation of the Unitree G1 (29-DoF) that serves the same DDS arm interface as the
real robot (`rt/arm_sdk` in, `rt/lowstate` out). Arm-control code (IK, trajectories, teleoperation) written
against the sim runs unchanged on the robot. Only the DDS domain and network interface differ.

```
 client ──rt/arm_sdk──▶ onboard controller ──rt/lowcmd──▶ unitree_mujoco ──rt/lowstate──▶ client
                              ▲                                                │
                              └──────────────── rt/lowstate ───────────────────┘
```

* **Simulator**: stock [`unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco) bridge (pinned commit),
  run through `sim/run_sim.py` (headless mode, 500 Hz PD, CRC, programmatic elastic band).
* **Emulated onboard controller** (`controller/`): balances the legs with the `unitree_rl_gym` G1 policy and blends
  your `rt/arm_sdk` waist/arm targets over an "arms down" default pose. It is the only publisher of `rt/lowcmd`.
* **Startup is automatic**: DAMP → STAND (lowered from the elastic band) → BALANCE (policy) → READY (band released).
  READY is reached about 10 s after the container starts. `rt/arm_sdk` is ignored before READY.

## Host prerequisites

* Linux, an NVIDIA GPU with the proprietary driver, and **Podman** (tested with 5.7).
* NVIDIA Container Toolkit with a CDI spec: `sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml`
  (the launcher passes the GPU as `--device nvidia.com/gpu=all`).
* For the viewer: an X11 session and local access for containers: `xhost +local:` (undo with `xhost -local:`).
  The viewer is watch-only; nothing in it is needed for normal operation.

## Launch and stop

```bash
./scripts/run.sh                 # builds g1-sim:dev if absent, then starts sim + controller with the viewer
HEADLESS=1 ./scripts/run.sh      # no viewer, no X11/GPU needed (tests, servers)
Ctrl-C                           # or:  podman stop g1-sim   (both processes terminate; no DDS publishers remain)
```

The container uses `--network host`, DDS **domain 1** pinned to interface **`lo`** (`docker/cyclonedds.xml`), so
nothing is published on domain 0 and a real robot on the same machine's network is not disturbed.
If the display is not accessible the launcher stops with instructions (`xhost +local:`). Controller state
transitions (BOOT, DAMP, STAND, BALANCE, READY, and falls) are printed to the container log.
If either process dies the container exits non-zero and names the failed one.

`compose.yaml` is provided as an equivalent for `podman-compose`; `scripts/run.sh` is the supported path.

## Client contract

| | Simulation | Real robot |
|---|---|---|
| `ChannelFactoryInitialize` | `(1, "lo")` | `(0, "<ethernet interface>")` |
| Publish | `rt/arm_sdk` (`LowCmd_`) only | `rt/arm_sdk` only |
| Subscribe | `rt/lowstate` (`LowState_`) | `rt/lowstate` |

* `motor_cmd[29].q` is the blend weight `w` (clamped to [0, 1]); joints 12-14 are the waist, 15-21 left arm, 22-28
  right arm. Per joint: `q = w·q_cmd + (1−w)·q_default`, `dq`/`tau` are scaled by `w`, and your `kp`/`kd` apply
  while `w > 0`. A joint with `kp = kd = tau = 0` is treated as uncommanded and held at the default pose.
* Leg indices 0-11 in `rt/arm_sdk` are ignored.
* **Never publish `rt/lowcmd` while the onboard controller runs**: two publishers fight over the motors.
* If your stream stops for 0.5 s while `w > 0`, the controller ramps `w` to 0 over 1 s (arms return to the
  default pose). This is a sim-side safety choice; the real robot's behavior is undocumented.
* Ramp `w` from 0 to 1 yourself (see `examples/arm_sdk_example.py`) while holding the current arm pose, as on hardware.

Try it: with the sim running, `python3 examples/arm_sdk_example.py` (needs `unitree_sdk2py`; or run it inside the
container with the repo mounted). On a real robot: `--domain 0 --iface <iface>`.

## Controller / simulator lockstep

The balance policy was trained without actuation latency. Two processes exchanging `rt/lowstate` / `rt/lowcmd` over DDS
add at least one physics step (2 ms) of delay, and a one-step delay alone raised the wandering from about 0.14 to
0.36 m/s. The sim therefore publishes `rt/lowstate` right after every physics step (exactly 500 Hz of sim time) and,
while the controller is running, waits up to 1.8 ms for its answer before the next step. The controller marks its
answer by echoing the state's tick in `LowCmd_.reserve[0]`. Clients are unaffected. A raw `rt/lowcmd` publisher
without that echo is never waited for, and the wait is suspended if the controller stops answering. The sim logs the
number of late answers on exit (normally about 0.1 %).

## Fidelity limits

* The balance controller is an **open approximation** of Unitree's proprietary one: the `unitree_rl_gym` G1 policy
  (legs only, trained with fixed arms, zero velocity command). Balance under fast or large arm motions differs from
  hardware.
* The gait-phase policy **marches in place and wanders**: with zero command the base moves at about 0.1-0.15 m/s
  along a slow arc (measured: about 8 m of travel per minute, up to 2.5 m from the start). It stays balanced, but it
  does not hold its position. Anything that needs a fixed base position must account for this.
* Measured in simulation (`controller/policy/README.md`): moderate full-range arm motion keeps the robot upright;
  aggressive swings (about ±1.5 rad at 1 Hz) eventually tip it over. The controller then **drops to damping on all joints
  and logs `FALL detected`** (threshold 0.6 rad roll/pitch) and stays damped until the container is restarted.
* Holding the arms far forward needs a hip-pitch compensation scheduled on the arm pose (see the policy notes); extreme
  poses may still fall.
* Unsupported: locomotion commands (`LocoClient`), RPC services (`MotionSwitcher`, `G1ArmActionClient`), hands.

## Swapping the balance policy

Implement `controller.policy.Policy` (`reset(state)`, `step(state) -> 12 leg targets`, plus `kp`, `kd`, `default_q`)
and instantiate it in `controller/main.py` instead of `RlGymG1Policy`. The `rt/arm_sdk` blending is independent of the
policy, so the client contract does not change. A policy trained with arm disturbance (e.g. a 29-DoF `unitree_rl_lab`
policy exported to TorchScript or ONNX) is the intended upgrade for better robustness.

## Tests

```bash
./tests/run_all.sh     # whole suite, headless, ~12 min: unit tests, sim checks, launch/stop, E2E, 5 min idle
```

Layout: `docker/` image + supervisor, `sim/` simulator entry point and band, `controller/` onboard controller,
`examples/` client, `scripts/` launcher and policy spike, `tests/`.
