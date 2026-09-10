#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(cd -- "${script_dir}/.." && pwd -P)"
image="${FORKLIFT_ROS2_IMAGE:-forklift/ros2-dev:jazzy}"
host_uid="$(id -u)"
host_gid="$(id -g)"

docker_args=(
    run
    --rm
    --init
    --interactive
    --network bridge
    --cap-drop ALL
    --security-opt no-new-privileges
    --user "${host_uid}:${host_gid}"
    --workdir /workspace
    --mount "type=bind,src=${repo_root},dst=/workspace"
    --env HOME=/tmp/forklift-home
    --tmpfs "/tmp/forklift-home:rw,nosuid,nodev,size=64m,uid=${host_uid},gid=${host_gid},mode=0700"
)

if [[ -t 0 && -t 1 ]]; then
    docker_args+=(--tty)
fi

if (( $# == 0 )); then
    set -- bash
fi

exec docker "${docker_args[@]}" "${image}" "$@"
