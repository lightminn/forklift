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

# Bring-up 7a (docs/design/2026-11-12-real-scan-obstacle-layer.md 1.5): the A2M12 driver and the scan adapter, with
# only that device and the scan socket's folder passed in. The host binds the socket (tools/scan_check.py or the bridge).
if [[ "${1:-}" == "scan-a2m12" ]]; then
    device="${2:-}"
    sock_dir="${3:-/tmp/forklift-scan}"
    if [[ -z "${device}" || ! -e "${device}" ]]; then
        echo "usage: tools/ros2_dev.sh scan-a2m12 <device, e.g. /dev/ttyUSB0> [socket folder, /tmp/forklift-scan]" >&2
        echo "no such device: '${device}'" >&2
        exit 2
    fi
    mkdir -p "${sock_dir}"
    image="${FORKLIFT_ROS2_SCAN_IMAGE:-forklift/ros2-scan:jazzy}"  # deploy/ros2/Dockerfile.scan
    docker_args+=(
        --device "${device}:${device}"
        --group-add "$(stat -L -c %g "${device}")"  # the device's own group, through a by-id link
        --mount "type=bind,src=${sock_dir},dst=/run/forklift-scan"
    )
    driver="ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=${device} -p serial_baudrate:=256000"
    driver+=" -p frame_id:=laser -p angle_compensate:=true -p scan_mode:=Sensitivity -r scan:=/sensors/lidar/scan_raw"
    adapter="python3 ros2/src/forklift_ros/forklift_ros/real_scan_adapter.py --socket /run/forklift-scan/scan.sock"
    # the driver in the background of this shell (sourced here first), the adapter in front; on exit the whole
    # process group goes (ros2 run's node is its child)
    set -- bash -c "source /opt/sllidar_ws/install/setup.bash || exit 1; trap 'kill 0' EXIT; ${driver} & ${adapter}"
fi

if [[ -t 0 && -t 1 ]]; then
    docker_args+=(--tty)
fi

if (( $# == 0 )); then
    set -- bash
fi

exec docker "${docker_args[@]}" "${image}" "$@"
