"""Tasks 2.4 / 2.5 / 2.6 against the running sim (no controller running).

2.4: PD-command one arm joint via rt/lowcmd and observe it move in rt/lowstate.
2.5: sim time vs wall time over 10 s within +-5 %.
2.6: no DDS traffic on domain 0.
"""
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

JOINT = 22  # right shoulder pitch
ok = True


def check(name, cond, detail=""):
    global ok
    ok &= bool(cond)
    print(("PASS " if cond else "FAIL ") + name, detail)


def main():
    ChannelFactoryInitialize(1, "lo")
    sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init()
    pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()
    crc = CRC()
    first = sub.Read(3.0)
    q0 = first.motor_state[JOINT].q
    target = q0 + 0.6
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_machine = first.mode_machine
    t_wall0, t_sim0 = time.time(), first.tick / 1000.0
    last = first
    while time.time() - t_wall0 < 10.0:
        cmd.mode_pr = 0
        for i in range(29):  # hold everything else limp-ish; drive only JOINT
            cmd.motor_cmd[i].mode = 1
        cmd.motor_cmd[JOINT].q, cmd.motor_cmd[JOINT].kp, cmd.motor_cmd[JOINT].kd = target, 40.0, 1.0
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)
        s = sub.Read(0.1)
        if s is not None:
            last = s
        time.sleep(0.002)
    wall = time.time() - t_wall0
    sim = last.tick / 1000.0 - t_sim0
    qf = last.motor_state[JOINT].q
    check("2.4 joint moved toward target", abs(qf - q0) > 0.3, f"(q0={q0:.3f} -> {qf:.3f}, target {target:.3f})")
    check("2.5 real-time pacing +-5%", abs(sim / wall - 1) <= 0.05, f"(sim {sim:.2f}s / wall {wall:.2f}s)")

    # 2.6: domain 0 must stay silent
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    dp = DomainParticipant(0)
    rd = DataReader(dp, Topic(dp, "rt/lowstate", LowState_))
    time.sleep(3.0)
    check("2.6 no rt/lowstate on domain 0", len(rd.take(10)) == 0)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
