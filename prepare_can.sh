#!/bin/bash
# Explicit mode switch only; never sends motor enable/reset/movement frames.
set -euo pipefail
mode=record
explicit=false
devices=()
for arg in "$@"; do
  case "$arg" in
    --mode=record|--mode=deploy|--mode=teleop) mode="${arg#*=}"; explicit=true ;;
    --help|-h) echo 'Usage: prepare_can.sh [--mode=record|deploy|teleop] [can0 can1 can2 can3]'; exit 0 ;;
    can[0-9]*) devices+=("$arg") ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done
# Independent arms require normal CAN ACKs even for receive-only ROS recording.
# Deployment needs only followers; collection/teleop needs all four interfaces.
if ((${#devices[@]} == 0)); then
  mapfile -t devices < <(/usr/bin/python3 - /home/robot/AgileX_ws/src/hardware_bringup/config/hardware.yaml "$mode" <<'PY'
import sys,yaml
with open(sys.argv[1]) as f: cfg=yaml.safe_load(f)
for section in (('arms',) if sys.argv[2]=='deploy' else ('arms','leaders')):
    for arm in cfg.get(section,{}).values(): print(arm['channel'])
PY
)
fi
((${#devices[@]})) || { echo 'No CAN interfaces configured' >&2; exit 1; }
for device in "${devices[@]}"; do
  [[ "$device" =~ ^can[0-9]+$ ]] || { echo "Invalid CAN interface: $device" >&2; exit 2; }
  info="$(ip -details link show "$device")"
  active=false
  [[ "${info%%$'\n'*}" =~ \<[^\>]*UP ]] && active=true
  match=false
  if [[ "$info" == *"bitrate 1000000"* ]]; then
    if [[ "$info" != *LISTEN-ONLY* ]]; then
      match=true
    fi
  fi
  if $active && $match; then
    echo "$device already UP at 1 Mbps, mode=$mode"
    continue
  fi
  if $active && ! $explicit; then
    echo "$device active with another configuration. Stop hardware/deployment first, then explicitly select --mode=record, --mode=deploy or --mode=teleop." >&2
    exit 1
  fi
  if pgrep -f '[p]iper_state_bridge.py|[p]iper_teleop_node.py' >/dev/null; then
    echo 'Stop start_hardware.sh before reconfiguring CAN.' >&2
    exit 1
  fi
  exec {can_lock_fd}>"/tmp/agilex-controller-$(id -u)-$device.lock"
  flock -n "$can_lock_fd" || { echo "An SDK deployment controller owns $device; stop it first." >&2; exit 1; }
  if $active; then sudo ip link set dev "$device" down; fi
  listen=off
  sudo ip link set dev "$device" type can bitrate 1000000 listen-only "$listen"
  sudo ip link set dev "$device" up
  ip -details link show "$device"
  exec {can_lock_fd}>&-
done
