## Purpose

Emulates the physical G1's onboard controller. It keeps the simulated robot balanced on its legs and serves the `rt/arm_sdk` arm-control interface, so that arm-control clients behave the same in simulation as on the real robot.

## ADDED Requirements

### Requirement: Automatic startup to ready state
The controller SHALL bring the robot from spawn to a freely balancing state without user interaction: hold on the elastic band, lower the robot to the ground, stand in the default pose, activate the balance policy, then release the band. After the band is released, the controller SHALL be in the READY state.

#### Scenario: Startup completes
- **WHEN** the container starts
- **THEN** within 30 s the robot is standing on the ground without band support and the controller reports READY

#### Scenario: Arm commands ignored before READY
- **WHEN** a client publishes `rt/arm_sdk` with weight 1 before the controller is READY
- **THEN** the arms remain in the default arms-down pose

### Requirement: Learned leg balance
While in the BALANCE and READY states, the controller SHALL command the leg joints (indices 0–11) from a learned locomotion policy evaluated at 50 Hz with zero velocity command, keeping the robot upright.

#### Scenario: Standing idle
- **WHEN** the controller is READY and no arm commands are received
- **THEN** the robot remains upright for at least 5 minutes

#### Scenario: Moderate arm motion
- **WHEN** a client moves both arms through their range at moderate speed via `rt/arm_sdk`
- **THEN** the robot remains upright

### Requirement: arm_sdk command interface
The controller SHALL subscribe to `rt/arm_sdk` using the Unitree HG `LowCmd_` message type. It SHALL interpret `motor_cmd[29].q` as a blend weight `w`, clamped to [0, 1], and `motor_cmd[12..28]` as targets for the waist and arm joints.

#### Scenario: Full weight
- **WHEN** a client publishes `rt/arm_sdk` with `w = 1` and target `q` for joints 15–28
- **THEN** the arm joints track the client's targets using the client's `kp`/`kd`

#### Scenario: Zero weight
- **WHEN** a client publishes `rt/arm_sdk` with `w = 0`
- **THEN** the arms are held in the default arms-down pose and the waist at zero

#### Scenario: Partial weight
- **WHEN** a client publishes `w = 0.5`
- **THEN** each commanded waist/arm target equals the midpoint between the client's target and the default pose

#### Scenario: Out-of-range weight
- **WHEN** a client publishes `w = 1.7` or `w = −0.3`
- **THEN** the controller treats the weight as 1 or 0 respectively

#### Scenario: Uncommanded joint
- **WHEN** a client publishes `w = 1` with `kp = kd = tau = 0` for a waist or arm joint (for example an arm-only client leaving the waist unset)
- **THEN** that joint stays at the default pose with default gains instead of going limp

#### Scenario: Leg commands ignored
- **WHEN** a client's `rt/arm_sdk` message contains nonzero commands for joints 0–11
- **THEN** those commands have no effect; legs remain under policy control

### Requirement: Arm command stream loss
If no `rt/arm_sdk` message is received for 0.5 s while `w > 0`, the controller SHALL ramp the effective weight to 0 over 1 s, returning the arms to the default pose.

#### Scenario: Client stops publishing
- **WHEN** a client publishing with `w = 1` stops publishing
- **THEN** after 0.5 s the arms move smoothly to the arms-down pose, reaching it within a further 1 s

### Requirement: Sole low-level command publisher
The controller SHALL publish `rt/lowcmd` to the simulation at approximately 500 Hz. Each message SHALL have the `mode_machine` value echoed from the latest `rt/lowstate` and a valid CRC.

#### Scenario: Command stream
- **WHEN** the controller is running
- **THEN** `rt/lowcmd` messages are published at approximately 500 Hz with valid CRCs

### Requirement: Fall protection
If the robot's base roll or pitch exceeds a configured threshold, the controller SHALL stop applying the policy, switch all joints to damping mode, and log an error.

#### Scenario: Robot falls
- **WHEN** a client drives the arms aggressively and the robot tips past the threshold
- **THEN** the controller enters damping mode and logs that a fall was detected

### Requirement: Observable controller state
The controller SHALL log its state transitions (BOOT, DAMP, STAND, BALANCE, READY, fall) to the container's standard output.

#### Scenario: State transition logged
- **WHEN** the controller transitions from BALANCE to READY
- **THEN** a log line indicating READY appears in the container logs
