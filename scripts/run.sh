#!/usr/bin/env bash
# Build (if absent) and start the G1 sim container with podman.
#   ./scripts/run.sh            # viewer on host X11 display, NVIDIA GPU
#   HEADLESS=1 ./scripts/run.sh # no viewer
#   ./scripts/run.sh bash       # any extra args replace the supervisor command
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE="${G1_SIM_IMAGE:-g1-sim:dev}"
HEADLESS="${HEADLESS:-0}"
ENGINE="${CONTAINER_ENGINE:-podman}"

command -v "$ENGINE" >/dev/null || { echo "error: $ENGINE not found" >&2; exit 1; }

if ! "$ENGINE" image exists "$IMAGE" 2>/dev/null; then
  echo "[run] building $IMAGE ..."
  "$ENGINE" build -t "$IMAGE" -f docker/Dockerfile .
fi

args=(run --rm --network host --security-opt label=disable
      -e HEADLESS="$HEADLESS" -e NVIDIA_DRIVER_CAPABILITIES=all
      --name g1-sim)

if [[ "${GPU:-1}" != "0" ]]; then
  if ls /etc/cdi/nvidia.yaml /var/run/cdi/nvidia.yaml >/dev/null 2>&1; then
    args+=(--device nvidia.com/gpu=all)
  else
    echo "warning: no NVIDIA CDI spec; run 'sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml'" >&2
  fi
fi

if [[ "$HEADLESS" == "0" ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "error: DISPLAY is not set; use HEADLESS=1 or run in an X11 session" >&2; exit 1
  fi
  if ! xhost 2>/dev/null | grep -qiE 'LOCAL:|SI:localuser'; then
    echo "error: container cannot access the X11 display." >&2
    echo "       grant access with:  xhost +local:    (revoke later: xhost -local:)" >&2
    echo "       or run without a viewer: HEADLESS=1 $0" >&2
    exit 1
  fi
  args+=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
fi

# Pass through G1_* tuning variables (e.g. G1_HIP_PITCH_BIAS).
while IFS= read -r v; do args+=(-e "$v"); done < <(env | grep -E '^G1_(HIP|DOMAIN|INTERFACE|BAND)' || true)

# Dev convenience: G1_DEV=1 mounts the working tree over the baked-in code.
if [[ "${G1_DEV:-0}" == "1" ]]; then
  args+=(-v "$PWD/sim:/app/sim:ro" -v "$PWD/controller:/app/controller:ro"
         -v "$PWD/docker/entrypoint.py:/app/entrypoint.py:ro"
         -v "$PWD/tests:/app/tests:ro" -v "$PWD/examples:/app/examples:ro")
fi

exec "$ENGINE" "${args[@]}" "$IMAGE" "$@"
