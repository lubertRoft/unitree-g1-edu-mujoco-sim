#!/usr/bin/env bash
# Full headless test suite (task 8.6). Uses the baked image g1-sim:dev; tests are mounted read-only.
set -u
cd "$(dirname "$0")/.."
IMG=${G1_SIM_IMAGE:-g1-sim:dev}
M=(-v "$PWD/tests:/app/tests:ro" -v "$PWD/examples:/app/examples:ro")
fails=0
NET=(--network host --security-opt label=disable)

step() { echo; echo "### $*"; }
chk()  { if "$@"; then :; else echo "FAILED: $*"; fails=$((fails+1)); fi; }
tc()   { podman run --rm "${NET[@]}" "${M[@]}" "$IMG" python3 "$@" 2>&1 | grep -vE "multicast|pygame|Hello from|FutureWarning|warnings.warn"; return "${PIPESTATUS[0]}"; }

start_stack() {  # $@ = optional command (default: supervisor => sim + controller)
  podman rm -f g1-sim >/dev/null 2>&1
  HEADLESS=1 ./scripts/run.sh "$@" >/tmp/g1-stack.log 2>&1 &
  if [[ $# -eq 0 ]]; then
    for _ in $(seq 1 300); do grep -q "BALANCE -> READY" /tmp/g1-stack.log && return 0; sleep 0.2; done
    echo "stack did not reach READY"; tail -20 /tmp/g1-stack.log; return 1
  fi
  sleep 8
}
stop_stack() { podman stop -t 5 g1-sim >/dev/null 2>&1; podman rm -f g1-sim >/dev/null 2>&1; }

step "unit tests (blending, stream loss, state machine)"
chk podman run --rm -v "$PWD:/w:ro" -w /w -e PYTHONPATH=/w "$IMG" python3 -m pytest -q -p no:cacheprovider tests/test_blend.py tests/test_machine.py

step "simulator alone: lowstate, lowcmd, pacing, domain 0, band"
start_stack python3 /app/sim/run_sim.py
chk tc /app/tests/smoke_lowstate.py
chk tc /app/tests/sim_lowcmd.py
chk bash -c "podman exec -i g1-sim python3 - < tests/band_headless.py 2>&1 | grep -vE multicast; exit \${PIPESTATUS[0]}"
stop_stack

step "launch / stop"
chk ./tests/launch_e2e.sh

step "controller + example client"
start_stack || fails=$((fails+1))
chk tc /app/tests/check_lowcmd.py
chk tc /app/tests/e2e_arm_sdk.py tracking
chk tc /app/tests/e2e_arm_sdk.py killed
chk tc /app/tests/e2e_arm_sdk.py motion
stop_stack

step "aggressive motion -> fall protection"
start_stack || fails=$((fails+1))
chk tc /app/tests/e2e_arm_sdk.py aggressive
chk grep -q "FALL detected" /tmp/g1-stack.log && echo "PASS fall error logged"
stop_stack

step "idle 5 min"
start_stack || fails=$((fails+1))
chk tc /app/tests/e2e_arm_sdk.py idle
stop_stack

echo; [[ $fails -eq 0 ]] && echo "ALL TESTS PASSED" || echo "$fails FAILURE(S)"
exit $fails
