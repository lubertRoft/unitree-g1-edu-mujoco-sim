## 1. Container Foundation

- [x] 1.1 Create project layout (`docker/`, `controller/`, `sim/`, `scripts/`, `examples/`, `tests/`) and a top-level README stub
- [x] 1.2 Write Dockerfile: Ubuntu base with NVIDIA GL support, Python, MuJoCo, CycloneDDS, `unitree_sdk2_python`, CPU-only PyTorch
- [x] 1.3 Clone `unitree_mujoco` at a pinned commit and `unitree_rl_gym` pretrained G1 policy at a pinned commit during the image build
- [x] 1.4 Add a CycloneDDS config that pins DDS to interface `lo` and set it via `CYCLONEDDS_URI` in the image
- [x] 1.5 Write the launch script (and compose file): build if absent, `--network host`, `--gpus all`, `DISPLAY` + `/tmp/.X11-unix` mounts, `HEADLESS` env flag, X11-access check with an explanatory error
- [x] 1.6 Verify: the container starts and `glxinfo` (or a MuJoCo test render) inside it reports the NVIDIA renderer

## 2. Simulator Configuration (stock unitree_mujoco)

- [x] 2.1 Configure the `simulate_python` config: robot `g1`, 29-DoF scene, domain `1`, interface `lo`, elastic band enabled, viewer on/off from `HEADLESS`
- [x] 2.2 Verify spawn: the robot appears upright on the elastic band in the viewer
- [x] 2.3 Write a host-side smoke script that subscribes to `rt/lowstate` (domain 1, `lo`), checks rate ≈500 Hz, valid CRC, and joint order 0–28 as in the hardware layout
- [x] 2.4 Verify `rt/lowcmd` is applied: a script commands one arm joint with PD and observes the motion in `rt/lowstate`
- [x] 2.5 Measure real-time pacing over 10 s (sim time vs wall time within ±5 %)
- [x] 2.6 Verify no traffic appears on DDS domain 0 while the sim runs

## 3. Programmatic Elastic Band Control

- [x] 3.1 Inspect how upstream lowers and releases the band via viewer key callbacks at the pinned commit
- [x] 3.2 Implement a minimal entry-point wrapper or a build-time patch exposing "lower band" and "release band" through a local signal (e.g. file, socket or env-driven channel) usable by the controller
- [x] 3.3 Verify: lowering and releasing work in headless mode with no keyboard input

## 4. Policy Spike (de-risk before building the controller)

- [x] 4.1 Read the upstream `g1.yaml` and deploy code: record the observation layout, default joint angles, PD gains, action scale, control decimation, and gait-phase period in `controller/policy/README.md`
- [x] 4.2 Load `motion.pt` on CPU and run a single inference with a dummy observation of the documented shape
- [x] 4.3 Prototype a script: band-held stand → lower → policy on legs (arms PD-held down) → release band; record whether the robot stays upright for 60 s
- [x] 4.4 Prototype arm disturbance: swing both arms at moderate speed while balancing; record whether the robot stays upright and whether it marches in place
- [x] 4.5 Write down the spike results and decide: proceed with `rl_gym` policy, or tune gains/default pose (escalate to the user if the policy is unusable)

## 5. Onboard Controller Core

- [x] 5.1 Create the controller package: config (indices, default pose, gains, thresholds, rates), DDS setup on domain 1 / `lo`, lowstate subscriber, lowcmd publisher with CRC
- [x] 5.2 Implement the 500 Hz publish loop with `mode_machine` echoed from the latest lowstate
- [x] 5.3 Implement the policy plug-in interface (observation builder, inference, action → leg targets) and the `rl_gym` G1 policy implementation running at 50 Hz with zero velocity command
- [x] 5.4 Implement the startup state machine BOOT → DAMP → STAND → BALANCE → READY, driving band lowering/release from section 3
- [x] 5.5 Implement fall protection: roll/pitch threshold → damping mode on all joints + error log
- [x] 5.6 Log all state transitions to stdout

## 6. arm_sdk Interface

- [x] 6.1 Subscribe to `rt/arm_sdk` (`LowCmd_`), keeping the latest message and its arrival timestamp
- [x] 6.2 Implement blending for joints 12–28: clamp `w`, blend `q` with the arms-down/waist-zero default, scale `dq`/`tau` by `w`, select gains
- [x] 6.3 Ignore arm_sdk input before READY, and ignore leg indices 0–11 in arm_sdk messages
- [x] 6.4 Implement stream-loss handling: no message for 0.5 s with `w > 0` → ramp effective `w` to 0 over 1 s
- [x] 6.5 Unit-test the blending and stream-loss logic (w = 0, 0.5, 1, out-of-range, leg commands ignored, timeout ramp) without DDS

## 7. Container Integration

- [x] 7.1 Write the supervisor entrypoint: start the sim, wait for `rt/lowstate`, start the controller; exit non-zero and name the failed process if either dies
- [x] 7.2 Verify the single-command launch from a clean checkout reaches READY within 30 s
- [x] 7.3 Verify stopping the container leaves no sim DDS publishers on the host

## 8. Examples and End-to-End Validation

- [x] 8.1 Add an example client (adapted from `g1_arm7_sdk_dds_example`) that ramps `w` 0 → 1, moves both arms, ramps back to 0, using `ChannelFactoryInitialize(1, "lo")`
- [x] 8.2 E2E: run the example and verify arms track targets at w = 1 and return to arms-down at w = 0
- [x] 8.3 E2E: kill the example mid-motion and verify the arms return to the default pose within 1.5 s
- [x] 8.4 E2E: idle READY robot remains upright for 5 minutes
- [x] 8.5 E2E: moderate full-range arm motion keeps the robot upright; aggressive motion triggers fall protection with a logged error
- [x] 8.6 Run the full test suite in headless mode

## 9. Documentation

- [x] 9.1 README: host prerequisites (Docker, NVIDIA container toolkit, `xhost +local:`), launch/stop, headless mode
- [x] 9.2 Document the client contract: domain 1 / `lo` in sim vs domain 0 / eth iface on the robot, publish only `rt/arm_sdk`, never `rt/lowcmd` alongside the controller
- [x] 9.3 Document fidelity limits (approximate balance controller, possible marching in place, arm-motion robustness) and how to swap in another policy
