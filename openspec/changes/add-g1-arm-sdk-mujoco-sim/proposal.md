## Why

Developing arm-control software (IK, trajectories, teleoperation) directly on the physical Unitree G1 EDU (29-DoF) is slow and risky. We need a local, containerized MuJoCo simulation that serves the same DDS arm interface as the real robot (`rt/arm_sdk` + `rt/lowstate`), so that client code written against the sim runs unchanged on hardware, differing only by DDS domain ID and network interface.

## What Changes

- Add a Docker container image that runs the stock `unitree_mujoco` Python simulator, configured for the G1 29-DoF scene, DDS domain `1`, network interface `lo`, with the MuJoCo viewer forwarded to the host X11 display using the NVIDIA GPU (watch-only).
- Add an **emulated onboard controller** process (Python, `unitree_sdk2_python`) inside the container that stands in for the G1's proprietary locomotion controller:
  - subscribes `rt/arm_sdk` (client arm/waist commands) and `rt/lowstate` (sim state),
  - balances the robot on its legs with an open RL locomotion policy (initially `unitree_rl_gym` G1 `motion.pt`),
  - blends arm and waist targets by the arm_sdk weight (`motor_cmd[29].q`) against a default "arms down" pose,
  - publishes the only `rt/lowcmd` stream to the simulator (≈500 Hz, `mode_machine` echoed, CRC set).
- Add an automatic startup sequence: spawn standing on the elastic band → lower to ground → stand → policy balance → release band. Arm commands are accepted only after the robot is balancing freely.
- Add launch tooling (compose file / run script) so a single command starts the sim on the host with `--network host` and GPU/X11 access.

## Capabilities

### New Capabilities
- `sim-container`: Container image and launch tooling: networking (host network, DDS domain 1, `lo`), GPU + X11 viewer passthrough, process supervision of simulator and controller.
- `g1-mujoco-sim`: Configuration of the stock `unitree_mujoco` simulator for the G1 29-DoF robot: scene, DDS bridge (`rt/lowcmd` in, `rt/lowstate` out), elastic band, and how the band is released automatically.
- `arm-sdk-onboard-controller`: Emulated G1 onboard controller: startup state machine, RL leg balance policy, `rt/arm_sdk` weight blending with arms-down default, `rt/lowcmd` publishing semantics.

### Modified Capabilities
<!-- none: no existing specs -->

## Impact

- **New code**: Dockerfile, launch script/compose file, simulator config, and the onboard controller Python package. All new; the repository is currently empty.
- **External dependencies**: `unitreerobotics/unitree_mujoco` (vendored or pinned at build time), `unitree_sdk2_python` + CycloneDDS, MuJoCo, PyTorch (CPU is enough for inference), `unitree_rl_gym` pretrained G1 policy weights.
- **Host requirements**: Linux with Docker, NVIDIA container toolkit (RTX 3070), X11 display access (`xhost +local:`).
- **Client contract**: Clients use `ChannelFactoryInitialize(1, "lo")` and publish only `rt/arm_sdk`. They must not publish `rt/lowcmd` while the onboard controller is running.
- **Fidelity limits**: The balance policy is an open approximation of Unitree's proprietary controller. Balance under fast or large arm motions may differ from the real robot.
