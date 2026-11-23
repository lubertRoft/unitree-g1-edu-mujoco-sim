"""Controller constants. Values below marked (spike) are confirmed in controller/policy/README.md."""
import os

DOMAIN_ID = int(os.environ.get("G1_DOMAIN_ID", "1"))
INTERFACE = os.environ.get("G1_INTERFACE", "lo")

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_ARM_SDK = "rt/arm_sdk"

NUM_MOTORS = 29
LEG = slice(0, 12)
WAIST_ARMS = slice(12, 29)
ARM_SDK_WEIGHT_IDX = 29

# Rates
CONTROL_HZ = 500.0
POLICY_PERIOD_MS = 20           # 50 Hz policy, scheduled on the sim clock (matches rl_gym control_decimation 10 x 2 ms)

# Default pose. Legs from rl_gym g1.yaml; waist zero; arms down (spike-tunable).
LEG_DEFAULT = [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
               -0.1, 0.0, 0.0, 0.3, -0.2, 0.0]
WAIST_DEFAULT = [0.0, 0.0, 0.0]
ARM_DEFAULT = [0.0, 0.25, 0.0, 0.5, 0.0, 0.0, 0.0,      # left: shoulder p/r/y, elbow, wrist r/p/y
               0.0, -0.25, 0.0, 0.5, 0.0, 0.0, 0.0]     # right
DEFAULT_Q = LEG_DEFAULT + WAIST_DEFAULT + ARM_DEFAULT

# PD gains (legs from rl_gym; waist/arms: Unitree example-like values)
LEG_KP = [100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40]
LEG_KD = [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2]
WAIST_KP, WAIST_KD = [150.0, 150.0, 150.0], [5.0, 5.0, 5.0]
ARM_KP, ARM_KD = [40.0] * 14, [1.0] * 14
DEFAULT_KP = LEG_KP + WAIST_KP + ARM_KP
DEFAULT_KD = LEG_KD + WAIST_KD + ARM_KD
DAMP_KD = 2.0

# arm_sdk stream-loss handling
ARM_SDK_TIMEOUT = 0.5   # s without a message while w > 0
ARM_SDK_RAMP = 1.0      # s to ramp effective w to 0

# Fall protection
FALL_ROLL_PITCH = 0.6   # rad (~34 deg); spike: 0.31 in moderate motion, 0.77 when falling

# Startup (tunable)
BAND_SOCKET = os.environ.get("G1_BAND_SOCKET", "/tmp/g1_band.sock")
BAND_LOWER_SPEED = 0.05     # m/s
BAND_RELEASE_RAMP = 1.5     # s: ramp the band height down below the torso before release
STAND_SETTLE = 2.0          # s of PD-to-default before the policy starts
BALANCE_HOLD = 3.0          # s of policy balance (band still slack) before release

BAND_LOWER_DROP = 0.12      # m below the spawn height; spike: feet loaded and band slack (tunable)
DAMP_TIME = 0.5             # s in DAMP before STAND
STAND_RAMP = 2.0            # s to interpolate from the current pose to the default pose

# Feed-forward bias added to both hip-pitch targets (not seen by the policy observation), scheduled on how far
# forward the arms are (mean measured shoulder pitch, reaching full bias at -HIP_BIAS_FULL_AT rad). Spike: without it
# the rl_gym policy tips over with the arms held forward; but a constant bias makes the idle robot fall after ~3 min.
HIP_PITCH_BIAS = float(os.environ.get("G1_HIP_PITCH_BIAS", "0.15"))
HIP_BIAS_FULL_AT = 0.8      # rad of forward shoulder pitch (joints 15, 22) for the full bias
HIP_BIAS_FILTER = 0.5       # s, first-order low-pass on the scheduling variable
