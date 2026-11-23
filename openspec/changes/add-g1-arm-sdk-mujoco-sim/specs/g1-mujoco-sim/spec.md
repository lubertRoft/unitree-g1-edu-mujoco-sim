## Purpose

Defines the physics simulation of the Unitree G1 29-DoF robot and its low-level DDS interface (`rt/lowcmd` in, `rt/lowstate` out), matching the message types and joint layout of the physical robot.

## ADDED Requirements

### Requirement: G1 29-DoF robot model
The simulation SHALL model the Unitree G1 with 29 actuated joints (12 leg, 3 waist, 14 arm) on a floating base standing on a ground plane, with joint indices matching the physical robot's motor index layout.

#### Scenario: Joint layout matches hardware
- **WHEN** a client reads `rt/lowstate`
- **THEN** motor states 0–11 are leg joints, 12–14 are waist joints, 15–21 are left arm joints and 22–28 are right arm joints, in the same order as on the physical G1 29-DoF

### Requirement: Low-level DDS state publication
The simulation SHALL publish `rt/lowstate` using the Unitree HG `LowState_` message type, containing joint positions, velocities and estimated torques, IMU orientation, angular velocity and acceleration, and a valid CRC.

#### Scenario: State stream rate
- **WHEN** the simulation is running
- **THEN** `rt/lowstate` messages are published at approximately 500 Hz

#### Scenario: State reflects physics
- **WHEN** a joint moves in the simulation
- **THEN** the corresponding `motor_state` position and velocity in `rt/lowstate` change accordingly

### Requirement: Low-level DDS command consumption
The simulation SHALL subscribe to `rt/lowcmd` using the Unitree HG `LowCmd_` message type, and SHALL apply each motor command as a PD-plus-feedforward torque: `tau = tau_ff + kp·(q_des − q) + kd·(dq_des − dq)`.

#### Scenario: Command applied
- **WHEN** a `LowCmd_` message sets a nonzero `kp` and a target `q` for a joint
- **THEN** the simulated joint is driven towards that target

### Requirement: Elastic band support controllable without the viewer
The simulation SHALL provide an elastic band that can hold the robot's pelvis above the ground, be lowered, and be released. These actions SHALL be triggerable programmatically by the onboard controller, without keyboard input in the viewer.

#### Scenario: Robot held at spawn
- **WHEN** the simulation starts
- **THEN** the robot is spawned upright and held by the elastic band

#### Scenario: Programmatic release
- **WHEN** the onboard controller requests band release
- **THEN** the band applies no further force and the robot is supported only by its legs

### Requirement: Real-time pacing
The simulation SHALL advance simulated time at approximately wall-clock rate.

#### Scenario: Time pacing
- **WHEN** the simulation runs for 10 s of wall-clock time on the reference host
- **THEN** the simulated time advanced is 10 s ± 5 %
