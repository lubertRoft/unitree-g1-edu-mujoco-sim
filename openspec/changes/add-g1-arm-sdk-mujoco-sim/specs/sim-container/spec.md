## Purpose

Packages the G1 MuJoCo simulation and its emulated onboard controller as a single container that can be started with one command on a local Linux host with an NVIDIA GPU and an X11 display.

## ADDED Requirements

### Requirement: Single-command launch
The project SHALL provide one command that builds (if needed) and starts the simulation container, including the simulator and the emulated onboard controller.

#### Scenario: Start from a clean checkout
- **WHEN** the user runs the launch command on a host with Docker and the NVIDIA container toolkit
- **THEN** the image is built if absent, the container starts, and both the simulator and the onboard controller are running

#### Scenario: Stop
- **WHEN** the user stops the container
- **THEN** both processes terminate and no DDS publishers from the sim remain on the host

### Requirement: Host networking for DDS
The container SHALL use the host network namespace, so that DDS clients on the same machine can communicate with the simulation using domain ID `1` on interface `lo` with no further network configuration.

#### Scenario: Local client discovers the sim
- **WHEN** a client on the host initializes the Unitree SDK with domain `1` and interface `lo` and subscribes to `rt/lowstate`
- **THEN** it receives state messages from the simulation

#### Scenario: No interference with a real robot
- **WHEN** the simulation is running
- **THEN** it publishes no DDS traffic on domain `0`

### Requirement: GPU-rendered watch-only viewer
The container SHALL display the MuJoCo viewer on the host's X11 display with GPU-accelerated rendering. The viewer is used for watching only, and its interaction with the simulation SHALL NOT be required for normal operation.

#### Scenario: Viewer appears
- **WHEN** the container is launched with X11 access granted on the host
- **THEN** a MuJoCo window showing the G1 robot appears on the host display

#### Scenario: Missing X11 access
- **WHEN** the launch command runs and the container cannot access the host display
- **THEN** the launch fails with a message explaining how to grant X11 access

#### Scenario: Headless mode
- **WHEN** the container is launched with headless mode enabled
- **THEN** the simulation and controller run without opening a viewer window

### Requirement: Process supervision
The container SHALL run the simulator and the onboard controller together, and SHALL exit with a non-zero status if either process terminates unexpectedly.

#### Scenario: Controller crashes
- **WHEN** the onboard controller process exits unexpectedly
- **THEN** the container stops and reports which process failed
