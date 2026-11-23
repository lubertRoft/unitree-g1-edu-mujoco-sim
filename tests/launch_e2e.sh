#!/usr/bin/env bash
# Tasks 7.2 / 7.3: single-command launch reaches READY < 30 s; stop leaves no sim publishers.
set -u
cd "$(dirname "$0")/.."
podman rm -f g1-sim >/dev/null 2>&1
LOG=$(mktemp); start=$(date +%s.%N)
HEADLESS=1 ./scripts/run.sh >"$LOG" 2>&1 &
ready=""
for _ in $(seq 1 600); do
  if grep -q "state BALANCE -> READY" "$LOG"; then ready=$(date +%s.%N); break; fi
  sleep 0.1
done
[[ -n "$ready" ]] || { echo "FAIL 7.2: READY not reached"; tail -20 "$LOG"; exit 1; }
dt=$(echo "$ready - $start" | bc -l)
printf "7.2 time to READY: %.1f s\n" "$dt"
(( $(echo "$dt < 30" | bc -l) )) && echo "PASS 7.2" || { echo "FAIL 7.2"; exit 1; }

podman stop -t 10 g1-sim >/dev/null
sleep 1
if podman ps -a --format '{{.Names}}' | grep -qx g1-sim; then echo "FAIL 7.3: container still present"; exit 1; fi
# no lowstate samples from a dead sim, and no leftover sim/controller processes
if podman run --rm --network host --security-opt label=disable g1-sim:dev python3 - <<'PY' 2>/dev/null
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
ChannelFactoryInitialize(1, "lo"); s = ChannelSubscriber("rt/lowstate", LowState_); s.Init()
sys.exit(0 if s.Read(3.0) is None else 1)
PY
then echo "PASS 7.3 (no rt/lowstate after stop)"; else echo "FAIL 7.3: rt/lowstate still published"; exit 1; fi
pgrep -f "^python3 (/app/sim/run_sim.py|-m controller.main)" >/dev/null && { echo "FAIL 7.3: leftover process"; exit 1; } || echo "PASS 7.3 (no leftover processes)"
