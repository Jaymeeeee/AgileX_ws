#!/bin/bash
set -euo pipefail
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
config="$ws/src/model_deploy/config/act.yaml"
offline=false
previous=''
for arg in "$@"; do
  [[ "$previous" == --config ]] && config="$arg"
  case "$arg" in
    --config=*) config="${arg#*=}" ;;
    --offline|--offline=*) offline=true ;;
  esac
  previous="$arg"
done
config="${config/#\~/$HOME}"
if $offline; then
  inference_python="$(/usr/bin/python3 - "$config" <<'PY'
import pathlib, sys, yaml
with open(sys.argv[1]) as f:
    print(pathlib.Path(yaml.safe_load(f)['python']).expanduser())
PY
)"
  prefix="$(dirname -- "$(dirname -- "$inference_python")")"
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1 LD_LIBRARY_PATH="$prefix/lib" PATH="$prefix/bin:/usr/bin:/bin"
  export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 OMP_NUM_THREADS=4 TOKENIZERS_PARALLELISM=false
  exec "$inference_python" "$ws/src/model_deploy/scripts/inference_worker.py" --config "$config" "$@"
fi
# ROS setup scripts are not nounset-safe.
set +u
source "$ws/env.sh"
set -u
exec /usr/bin/python3 "$ws/src/model_deploy/scripts/deploy_node.py" --config "$config" "$@"
