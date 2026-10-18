#!/usr/bin/env bash
set -euo pipefail
training_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
lerobot_python="${LEROBOT_PYTHON:-$HOME/miniconda3/envs/lerobot/bin/python}"
if [[ ! -x "$lerobot_python" ]]; then
    echo "找不到 LeRobot Python: $lerobot_python" >&2
    exit 1
fi
lerobot_prefix="$(cd -- "$(dirname -- "$lerobot_python")/.." && pwd)"
unset PYTHONPATH
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export LD_LIBRARY_PATH="$lerobot_prefix/lib"
export PATH="$lerobot_prefix/bin:$PATH"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1
export TQDM_DISABLE=1 TOKENIZERS_PARALLELISM=false
# Refuse CPU fallback if the driver/environment becomes unavailable.
"$lerobot_python" -c 'import torch; assert torch.cuda.is_available(), "CUDA unavailable"; print("GPU:", torch.cuda.get_device_name(0))'
cd "$training_dir/.."
config_args=("--config_path=$training_dir/act.yaml")
for arg in "$@"; do
    case "$arg" in
        --config_path|--config_path=*) config_args=(); break ;;
    esac
done
exec "$lerobot_python" -m lerobot.scripts.lerobot_train \
    "${config_args[@]}" "$@"
