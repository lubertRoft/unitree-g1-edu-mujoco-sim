## Context

The repository is empty. Unitree publishes `unitree_mujoco`, a MuJoCo simulator with a DDS bridge. It subscribes `rt/lowcmd`, publishes `rt/lowstate`, ships a G1 29-DoF scene, and has an elastic band that acts as a virtual gantry. It does **not** serve `rt/arm_sdk`, and it has no equivalent of the G1's proprietary onboard locomotion controller. On the real robot, `rt/arm_sdk` commands are blended over that controller while it keeps the robot balanced. The sim therefore needs a component that plays the controller's role. See proposal.md for motivation.

G1 29-DoF motor index layout (shared by `LowCmd_`/`LowState_`, 35 slots):

```
 0–5    left leg   (hip pitch/roll/yaw, knee, ankle pitch/roll)
 6–11   right leg
 12–14  waist      (yaw, roll, pitch)
 15–21  left arm   (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)
 22–28  right arm
 29     unused motor slot; in rt/arm_sdk, motor_cmd[29].q = blend weight w ∈ [0,1]
```

## Goals / Non-Goals

**Goals:**
- Client code that targets `rt/arm_sdk` runs unchanged against sim and robot. Only `(domain_id, interface)` differs: sim `(1, "lo")`, robot `(0, "<eth iface>")`.
- Robot stands on the ground, balanced by a learned policy, with no human interaction after `start`.
- The balance policy can be swapped later without changing the arm_sdk contract.

**Non-Goals:**
- Reproducing Unitree's proprietary controller exactly. Fidelity of balance under aggressive arm motion is best-effort.
- Locomotion commands (walking, `LocoClient`), RPC services (`MotionSwitcher`, `G1ArmActionClient`), dexterous hand topics.
- An interactive viewer, remote or browser viewing, multi-robot setups.
- Running the sim on a machine other than the client's.

## Decisions

### D1: Use the stock `unitree_mujoco` Python simulator as the physics + DDS layer
Pin a specific upstream commit at image build time and configure it through its config file (robot `g1`, 29-DoF scene, domain 1, interface `lo`, elastic band on).
- *Why*: The user wants to use the sim's own DDS interface. Upstream already handles MJCF, sensor mapping and `LowState_` packing correctly.
- *Alternatives*: The C++ simulator (heavier build, same interface). A self-written MuJoCo loop (duplicates upstream, risks joint-order and message-packing bugs).

### D2: Emulate the onboard controller as a separate process between client and sim
```
 client ──rt/arm_sdk──▶ onboard_ctrl ──rt/lowcmd──▶ unitree_mujoco ──rt/lowstate──▶ client
                             ▲                                          │
                             └────────────── rt/lowstate ───────────────┘
```
The controller is the **only** publisher of `rt/lowcmd`.
- *Why*: This mirrors the real robot's architecture, where the client never touches legs and a hidden controller owns the motors. It also keeps the simulator stock, and lets the controller be disabled to fall back to raw `rt/lowcmd` use.
- *Alternatives*: Patching arm_sdk handling into the simulator (forks upstream, mixes concerns).

### D2b: Lockstep between simulator and controller (found during implementation)
The sim publishes `rt/lowstate` right after each physics step, and while the controller is running it waits (up to
1.8 ms) for the command that answers that state before stepping again. The controller echoes the state tick in
`LowCmd_.reserve[0]`; raw `rt/lowcmd` clients without the echo are never waited for.
- *Why*: The `rl_gym` policy assumes no actuation latency. Free-running processes over DDS add >= 1 physics step plus
  jitter, which measurably made the robot walk (0.3-0.45 m/s) instead of stand (0.14 m/s with the handshake) and could
  make it fall. In-process experiments showed a 1-step command delay alone reproduces the effect.
- *Alternatives*: Make the controller timer-driven and accept the delay (rejected: robot wanders); retrain the policy
  with latency (out of scope).

### D3: Balance policy = `unitree_rl_gym` pretrained G1 `motion.pt`, legs only, behind a policy interface
The policy runs at 50 Hz and produces leg joint targets (indices 0–11). Waist and arms are not policy outputs. They are driven by the blending logic (D4). The controller treats the policy as a plug-in with a defined observation builder and action mapping, so an `unitree_rl_lab` 29-DoF ONNX policy can replace it later.
- *Why*: These are the only official pretrained G1 weights shipped. CPU inference of a small MLP is cheap. Because the arms are not policy outputs, arm_sdk blending is conflict-free.
- *Alternatives*: The `unitree_rl_lab` G1-29dof policy (closer to the modern stack, but no weights shipped, so it needs Isaac Lab training first; deferred). A plain PD leg hold (falls under arm motion).
- *To verify in spike*: the exact observation vector (expected: ang vel, projected gravity, velocity command, 12 joint pos/vel, last action, gait phase sin/cos), the default joint angles, the PD gains and the action scale, all taken from the upstream `g1.yaml`.

### D4: arm_sdk blending semantics
For each waist/arm joint `i ∈ [12, 28]` at the 500 Hz control rate:
```
 w      = clamp(arm_sdk.motor_cmd[29].q, 0, 1)
 q_i    = w · arm_sdk.q_i  + (1 − w) · q_default_i      (q_default = arms-down pose, waist 0)
 dq_i   = w · arm_sdk.dq_i
 tau_i  = w · arm_sdk.tau_i
 kp_i, kd_i = arm_sdk kp/kd when w > 0 else default gains
```
If no `rt/arm_sdk` message arrives for 0.5 s, the controller ramps `w` to 0 over 1 s, so the arms return to the arms-down pose.
- *Why*: Linear blending with weight is how Unitree's arm_sdk examples ramp control in and out. The stream-loss fallback is a sim-side safety choice. The real robot's exact behavior on stream loss is undocumented.

### D5: Automatic startup state machine
```
 BOOT ─▶ DAMP ─▶ STAND (band holds, lower to ground, PD to default pose)
           ─▶ BALANCE (policy active) ─▶ READY (band released; arm_sdk accepted)
 any state ─(fall detected: |roll| or |pitch| > threshold)─▶ DAMP + error log
```
Before READY, `rt/arm_sdk` input is ignored and `w` is treated as 0.

### D6: Elastic band control without manual key presses
Upstream toggles and lowers the band only through viewer key callbacks. We add a minimal entry-point wrapper (or a small patch applied at build time against the pinned commit) that exposes band lowering and release to the controller's state machine through a simple local signal.
- *Why*: Fully automatic startup was required. Keeping the change as a small, documented patch against a pinned commit keeps the simulator "essentially stock".
- *Alternatives*: Spawning directly on the ground without the band (the policy may not catch the initial drop); simulating key events into the viewer (fragile).

### D7: Container runtime
One image with both processes, started by a small supervisor entrypoint that brings the sim up first and then the controller. If either process exits, the container exits. Run with `--network host` (DDS over loopback, no multicast setup across a bridge), `--gpus all` + NVIDIA container toolkit for GL, `DISPLAY` and `/tmp/.X11-unix` mounted for the passive viewer. PyTorch is installed as a CPU build to keep the image small; the GPU is only used for rendering.

## Risks / Trade-offs

- [The `rl_gym` policy was trained with fixed arms, so large or fast arm motions may destabilize it] → Document the limit. Fall detection drops to DAMP rather than producing garbage. The policy interface (D3) allows a swap to a 29-DoF policy trained with arm disturbance.
- [A gait-phase policy may step in place instead of standing still at zero command] → Accept for v1 and note it in the README. A standing-specific policy can be swapped in later.
- [Upstream `unitree_mujoco` changes break the band patch] → Pin the commit. The patch is small and covered by a startup smoke test.
- [Two processes are both on the 500 Hz loop, with timing jitter between them] → The controller publishes at a fixed rate from its own timer. The simulator applies the latest command, the same as the real robot does.
- [DDS on `lo`: CycloneDDS discovery on loopback can need explicit config] → Ship a CycloneDDS config that pins the interface to `lo`, and document the client-side equivalent.
- [Someone runs a client that publishes `rt/lowcmd` concurrently] → Two publishers produce conflicting commands. Document this in the client contract, and log a warning when foreign `rt/lowcmd` writers are detected, if feasible.
- [X11 access requires `xhost +local:`] → The launch script checks and explains. Headless mode (no viewer) is supported via an env flag for tests.

## Open Questions

- Exact numeric thresholds (fall detection angles, band lowering speed, default arm pose angles). These can be tuned during implementation without changing the specs.
