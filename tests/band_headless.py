"""Task 3.3: band lower/release work with no keyboard/viewer (sim must run HEADLESS=1)."""
import sys, time
sys.path.insert(0, "/app/sim")
from band import BandClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

ChannelFactoryInitialize(1, "lo")
sub = ChannelSubscriber("rt/sportmodestate", SportModeState_); sub.Init()
band = BandClient("/tmp/g1_band.sock")

def z():
    return sub.Read(2.0).position[2]

band.set_height(0.87); time.sleep(2.0); z_hold = z()
band.set_height(0.80); time.sleep(2.0); z_low = z()
band.release(); time.sleep(2.0); z_free = z()
band.set_height(0.87)  # restore
print(f"z hold={z_hold:.3f} lowered={z_low:.3f} released={z_free:.3f}")
ok = z_low < z_hold - 0.03 and z_free < z_low - 0.0 and not band.status()[0] is None
print("PASS 3.3" if ok else "FAIL 3.3"); sys.exit(0 if ok else 1)
