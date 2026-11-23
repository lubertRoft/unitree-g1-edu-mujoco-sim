"""Controller publishes rt/lowcmd at ~500 Hz with valid CRC and mode_machine echoed."""
import sys, time
sys.path.insert(0, "/app/tests")
import common
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

common.init()
sub = ChannelSubscriber("rt/lowcmd", LowCmd_); sub.Init()
ls_sub = common.lowstate_sub()
mm = ls_sub.Read(2.0).mode_machine
crc, n, bad, wrong_mm, t0 = CRC(), 0, 0, 0, None
end = time.time() + 5
while time.time() < end:
    m = sub.Read(1.0)
    if m is None: continue
    t0 = t0 or time.time(); n += 1
    bad += crc.Crc(m) != m.crc; wrong_mm += m.mode_machine != mm
rate = n / (time.time() - t0)
print(f"rate={rate:.0f} Hz bad_crc={bad} wrong_mode_machine={wrong_mm}")
ok = 450 <= rate <= 550 and bad == 0 and wrong_mm == 0
print("PASS lowcmd" if ok else "FAIL lowcmd"); sys.exit(not ok)
