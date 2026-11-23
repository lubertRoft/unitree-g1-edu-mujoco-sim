"""Policy spike (tasks 4.2-4.4): in-process MuJoCo test of the rl_gym G1 policy on the 29-DoF model.

  python3 spike_policy.py infer                # 4.2: dummy inference
  python3 spike_policy.py stand   [--seconds 60]   # 4.3: band -> lower -> policy -> release
  python3 spike_policy.py arms    [--seconds 40] [--amp A --freq F]   # 4.4: arm swing
"""
import argparse
import math
import sys

import mujoco
import numpy as np
import torch

XML = "/opt/unitree_mujoco/unitree_robots/g1/scene_29dof.xml"
POLICY = "/opt/unitree_rl_gym/deploy/pre_train/g1/motion.pt"
DEFAULT = np.array([-0.1, 0, 0, 0.3, -0.2, 0] * 2, dtype=np.float32)
KP = np.array([100, 100, 100, 150, 40, 40] * 2, dtype=np.float32)
KD = np.array([2, 2, 2, 4, 2, 2] * 2, dtype=np.float32)
WAIST_ARM_DEFAULT = np.array([0, 0, 0] + [0, 0.25, 0, 0.5, 0, 0, 0] + [0, -0.25, 0, 0.5, 0, 0, 0], dtype=np.float32)
WA_KP = np.array([150] * 3 + [40] * 14, dtype=np.float32)
WA_KD = np.array([5] * 3 + [1] * 14, dtype=np.float32)
DT = 0.002


def gravity(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


def build_obs(d, last_action, t):
    qj = d.qpos[7:19] - DEFAULT
    dqj = d.qvel[6:18] * 0.05
    ph = (t % 0.8) / 0.8
    obs = np.zeros(47, dtype=np.float32)
    obs[0:3] = d.qvel[3:6] * 0.25
    obs[3:6] = gravity(d.qpos[3:7])
    obs[6:9] = 0.0
    obs[9:21], obs[21:33], obs[33:45] = qj, dqj, last_action
    obs[45:47] = [math.sin(2 * math.pi * ph), math.cos(2 * math.pi * ph)]
    return obs


def tilt(d):
    g = gravity(d.qpos[3:7])
    return math.acos(max(-1, min(1, -g[2])))  # angle between body z and world z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["infer", "stand", "arms", "pose"])
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--amp", type=float, default=0.6)
    ap.add_argument("--freq", type=float, default=0.3)
    ap.add_argument("--pose-scale", type=float, default=1.0, help="pose mode: fraction of the example target pose")
    ap.add_argument("--ankle-bias", type=float, default=0.0, help="rad added to both ankle-pitch targets")
    ap.add_argument("--hip-bias", type=float, default=0.0)
    ap.add_argument("--delay-steps", type=int, default=0, help="observation latency in physics steps (2 ms each)")
    ap.add_argument("--t-policy", type=float, default=2.0, help="time the policy starts (stack order: 6.4)")
    ap.add_argument("--pd-delay", type=int, default=0, help="PD loop uses state this many physics steps old")
    ap.add_argument("--kp-scale", type=float, default=1.0)
    a = ap.parse_args()

    policy = torch.jit.load(POLICY)
    if a.mode == "infer":
        out = policy(torch.zeros(1, 47))
        print("inference ok, output shape", tuple(out.shape), "values", out.detach().numpy().round(3)[0])
        return 0

    m = mujoco.MjModel.from_xml_path(XML); d = mujoco.MjData(m); m.opt.timestep = DT
    d.qpos[2] = 0.82; d.qpos[7:19] = DEFAULT; d.qpos[19:] = WAIST_ARM_DEFAULT
    mujoco.mj_forward(m, d)
    torso = m.body("torso_link").id
    z0 = d.xpos[torso][2]
    band_h, band_on = z0, True
    action = np.zeros(12, dtype=np.float32)
    target = DEFAULT.copy()
    wa_target = WAIST_ARM_DEFAULT.copy()
    t_stand, t_policy, t_lower_end, t_release = 2.0, a.t_policy, None, None
    lower_speed = 0.05
    n = int(a.seconds / DT)
    max_tilt, upright_since_release, fell_at, steps_moved = 0.0, None, None, 0
    foot_x = []
    from collections import deque
    hist = deque(maxlen=max(a.delay_steps, a.pd_delay) + 1)
    dd = mujoco.MjData(m)
    path, last_xy = 0.0, None
    for k in range(n):
        t = k * DT
        hist.append((d.qpos.copy(), d.qvel.copy()))
        if t_release is not None:
            xy = d.qpos[:2].copy()
            if last_xy is not None and k % 50 == 0:
                path += float(np.linalg.norm(xy - last_xy))
            if k % 50 == 0:
                last_xy = xy
        # band
        if band_on:
            f = max(3000 * (band_h - d.xpos[torso][2]) - 300 * d.cvel[torso][5], 0)
            d.xfrc_applied[torso, :3] = [200 * (0 - d.xpos[torso][0]) - 50 * d.cvel[torso][3],
                                         200 * (0 - d.xpos[torso][1]) - 50 * d.cvel[torso][4], f]
        else:
            d.xfrc_applied[torso, :3] = 0
        # state machine
        if t >= t_policy and k % 10 == 0:
            with torch.no_grad():
                if a.delay_steps:
                    dd.qpos[:], dd.qvel[:] = hist[0]
                action = policy(torch.from_numpy(build_obs(dd if a.delay_steps else d, action, t)).unsqueeze(0)).numpy()[0]
            target = action * 0.25 + DEFAULT
        if t >= 4.0 and t_lower_end is None:      # lower the band until it goes slack
            band_h -= lower_speed * DT
            if band_h < z0 - 0.12:
                t_lower_end = t
        if t_lower_end is not None and t_release is None and t - t_lower_end > 3.0:
            band_on, t_release = False, t
            print(f"[{t:.1f}s] band released")
        if a.mode == "arms" and t_release is not None and t - t_release > 3.0:
            ph = 2 * math.pi * a.freq * (t - t_release)
            wa_target = WAIST_ARM_DEFAULT.copy()
            for idx in (15, 22):  # shoulder pitch both arms, relative to waist/arm array (idx-12)
                wa_target[idx - 12] += a.amp * math.sin(ph)
            for idx in (16, 23):
                wa_target[idx - 12] += 0.5 * a.amp * math.sin(ph + 1.0)
            wa_target[18 - 12] += 0.5 * a.amp * math.sin(ph)  # elbows
            wa_target[25 - 12] += 0.5 * a.amp * math.sin(ph)
        if a.mode == "pose" and t_release is not None and t - t_release > 3.0:
            f = 0.5 - 0.5 * math.cos(math.pi * min((t - t_release - 3.0) / 4.0, 1.0))
            tgt = np.array([0, 0, 0] + [-0.8, 0.5, 0, 0.7, 0, 0, 0] + [-0.8, -0.5, 0, 0.7, 0, 0, 0], dtype=np.float32)
            wa_target = WAIST_ARM_DEFAULT + f * a.pose_scale * (tgt - WAIST_ARM_DEFAULT)
            max_pitch = max(globals().get("max_pitch", 0), abs(math.asin(max(-1, min(1, -gravity(d.qpos[3:7])[0])))))
            globals()["max_pitch"] = max_pitch
        # PD
        bias = np.zeros(12, dtype=np.float32); bias[[4, 10]] = a.ankle_bias * (wa_target[3] - WAIST_ARM_DEFAULT[3] != 0 or a.mode == 'pose'); bias[[0, 6]] = a.hip_bias
        pq, pv = (hist[-1 - min(a.pd_delay, len(hist) - 1)] if a.pd_delay else (d.qpos, d.qvel))
        d.ctrl[0:12] = a.kp_scale * KP * (target + bias - pq[7:19]) - KD * pv[6:18]
        d.ctrl[12:29] = WA_KP * (wa_target - pq[19:36]) - WA_KD * pv[18:35]
        mujoco.mj_step(m, d)
        tl = tilt(d)
        if t_release is not None:
            max_tilt = max(max_tilt, tl)
            if (tl > 0.9 or d.qpos[2] < 0.45) and fell_at is None:
                fell_at = t - t_release
                print(f"[{t:.1f}s] FELL after {fell_at:.1f}s of free standing (tilt {tl:.2f}, z {d.qpos[2]:.2f})")
                break
        if k % int(5 / DT) == 0:
            print(f"[{t:5.1f}s] z={d.qpos[2]:.3f} tilt={tl:.3f} band={'on' if band_on else 'off'} xy=({d.qpos[0]:.2f},{d.qpos[1]:.2f})")
    drift = math.hypot(d.qpos[0], d.qpos[1])
    print(f"max_pitch={globals().get('max_pitch', 0):.2f}"); print(f"RESULT mode={a.mode} fell={fell_at is not None} max_tilt={max_tilt:.2f} rad final_z={d.qpos[2]:.3f} "
          f"base_drift={drift:.2f} m path={path:.1f} m ({path / max(a.seconds - (t_release or 0), 1):.2f} m/s) free_time={(a.seconds - (t_release or a.seconds)):.0f}s")
    return 0 if fell_at is None else 2


if __name__ == "__main__":
    sys.exit(main())
