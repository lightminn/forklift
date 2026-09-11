#!/bin/bash
#SBATCH --job-name=forklift-model-check
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=00:20:00

set -euo pipefail
umask 0007

if [[ "$#" -ne 6 && "$#" -ne 8 ]]; then
    echo "usage: model_check.sbatch MODE SOURCE OUTPUT PYTHON IMAGE DURATION [CATALOGUE RANGE]" >&2
    exit 2
fi

mode=$1
source_dir=$2
output_dir=$3
python_path=$4
image=$5
duration=$6
catalogue=${7:--}
scene_range=${8:--}

args=(
    --mode "$mode"
    --source "$source_dir"
    --output "$output_dir"
    --duration "$duration"
)
if [[ "$python_path" != "-" ]]; then
    args+=(--python "$python_path")
fi
if [[ "$image" != "-" ]]; then
    args+=(--image "$image")
fi

if [[ "$mode" == "scenes" ]]; then
    args+=(--catalogue "$catalogue" --scene-range "$scene_range"
           --run-id "$(basename -- "$source_dir")")
fi

export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 "$source_dir/tools/remote_model_job.py" "${args[@]}"
