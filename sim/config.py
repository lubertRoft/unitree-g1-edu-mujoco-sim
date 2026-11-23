"""Simulator configuration for the stock unitree_mujoco bridge.

Shadows upstream ``simulate_python/config.py`` (this directory comes first on
PYTHONPATH). The upstream bridge only reads ``ROBOT`` from here; the rest is
used by ``run_sim.py``.
"""
import os

UPSTREAM = os.environ.get("UNITREE_MUJOCO_DIR", "/opt/unitree_mujoco")

ROBOT = "g1"
ROBOT_SCENE = os.path.join(UPSTREAM, "unitree_robots", "g1", "scene_29dof.xml")
DOMAIN_ID = 1
INTERFACE = "lo"

USE_JOYSTICK = 0
PRINT_SCENE_INFORMATION = False

# 500 Hz physics == 500 Hz rt/lowstate (spec).
SIMULATE_DT = 0.002
VIEWER_DT = 0.02

# HEADLESS=1 disables the viewer (no X11 / GPU needed).
HEADLESS = os.environ.get("HEADLESS", "0") not in ("", "0", "false", "False")

# Real G1 29-DoF reports mode_machine=5 in LowState_; the controller echoes it.
MODE_MACHINE = 5

# Local control channel for the elastic band (controller -> sim).
BAND_SOCKET = os.environ.get("G1_BAND_SOCKET", "/tmp/g1_band.sock")

# Spawn: default (policy) leg pose, pelvis raised so the feet hang just above
# the floor, robot held by the band.
SPAWN_PELVIS_Z = 0.82
SPAWN_LEG_ANGLES = [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0]

# Elastic band: one-sided vertical PD on the torso (only pulls up) plus a weak
# horizontal centering spring. `height` is the commanded torso height.
BAND_KZ = 3000.0
BAND_DZ = 300.0
BAND_KXY = 200.0
BAND_DXY = 50.0
BAND_ATTACH_BODY = "torso_link"
