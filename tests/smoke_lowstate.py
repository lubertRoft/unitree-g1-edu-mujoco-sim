"""Subscribe to rt/lowstate (domain 1, lo): rate, CRC, joint layout (task 2.3).

Run from any container/host with unitree_sdk2py:  python3 tests/smoke_lowstate.py
"""
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC

# Expected hardware layout, G1 29-DoF (indices 0-28).
LAYOUT = (["l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll",
           "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll",
           "waist_yaw", "waist_roll", "waist_pitch"]
          + [f"l_{j}" for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                                "wrist_roll", "wrist_pitch", "wrist_yaw")]
          + [f"r_{j}" for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                                "wrist_roll", "wrist_pitch", "wrist_yaw")])


def main(duration=5.0):
    ChannelFactoryInitialize(1, "lo")
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()
    crc = CRC()
    n = bad_crc = 0
    ticks = []
    t0 = None
    end = None
    last = None
    while True:
        msg = sub.Read(2.0)
        if msg is None:
            print("FAIL: no rt/lowstate within 2 s"); return 1
        now = time.time()
        if t0 is None:
            t0, end = now, now + duration
        n += 1
        ticks.append(msg.tick)
        if crc.Crc(msg) != msg.crc:
            bad_crc += 1
        last = msg
        if now >= end:
            break
    wall = time.time() - t0
    rate = (n - 1) / wall
    sim_rate = (ticks[-1] - ticks[0]) / 1000.0 / wall  # sim s per wall s (informational)
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        print(("PASS " if cond else "FAIL ") + name, detail)

    check("rate ~500 Hz", 450 <= rate <= 550, f"({rate:.0f} Hz)")
    check("CRC valid", bad_crc == 0, f"({bad_crc}/{n} bad)")
    check("mode_machine set", last.mode_machine != 0, f"({last.mode_machine})")
    check("35 motor slots", len(last.motor_state) == 35)
    # Layout sanity: legs/waist/arms are present at 0-28; check the rest pose
    # moves plausibly (all joint positions finite and inside +-pi).
    qs = [last.motor_state[i].q for i in range(29)]
    check("joint positions finite", all(abs(q) < 3.2 for q in qs))
    # Actuator order in the MJCF is what the bridge maps to motor index i.
    try:
        import mujoco
        m = mujoco.MjModel.from_xml_path("/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml")
        names = [m.actuator(i).name for i in range(m.nu)]
        want = [n.replace("_joint", "") for n in names]
        pat = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]
        exp = [f"{side}_{p_}" for side in ("left", "right") for p_ in pat]
        exp += ["waist_yaw", "waist_roll", "waist_pitch"]
        arm = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
        exp += [f"{side}_{a}" for side in ("left", "right") for a in arm]
        check("MJCF actuator order == hardware layout 0-28", m.nu == 29 and want == exp,
              "" if want == exp else f"\n  got {want}\n  exp {exp}")
    except ImportError:
        print("SKIP mjcf layout check (mujoco not installed)")
    print("layout (idx:name) 0-28:", ", ".join(f"{i}:{n_}" for i, n_ in enumerate(LAYOUT)))
    print("INFO sim/wall tick ratio %.3f" % sim_rate)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
