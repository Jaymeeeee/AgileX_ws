#!/bin/bash
# Prepare receive-only USB-CAN interfaces; this does not enable or move any arm.
set -euo pipefail
for channel in "${@:-can0 can1}"; do
  # No argument means both interfaces; explicit arguments may also select one.
  for device in $channel; do
    [[ "$device" =~ ^can[0-9]+$ ]] || { echo "Invalid CAN interface: $device" >&2; exit 2; }
    info="$(ip -details link show "$device")"
    if [[ "${info%%$'\n'*}" =~ \<[^\>]*UP ]]; then
      if [[ "$info" == *LISTEN-ONLY* && "$info" == *"bitrate 1000000"* ]]; then
        echo "$device already UP at 1 Mbps in LISTEN-ONLY mode"
        continue
      fi
      echo "$device is already active with another configuration; inspect it before changing mode." >&2
      exit 1
    fi
    sudo ip link set dev "$device" type can bitrate 1000000 listen-only on
    sudo ip link set dev "$device" up
    ip -details link show "$device"
  done
done
