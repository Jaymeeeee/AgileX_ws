#!/bin/bash
set -euo pipefail
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "$ws/env.sh"
set -u
exec /usr/bin/python3 "$ws/src/model_deploy/scripts/replay_node.py" "$@"
