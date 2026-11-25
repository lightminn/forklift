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
    if [[ -n "${FORKLIFT_ROS2_SCAN_IMAGE:-}" ]]; then
        echo "scan image overridden by FORKLIFT_ROS2_SCAN_IMAGE: ${image}" >&2
    fi
    # the image that runs, by ID (7a-3 plan 3: a timing bound is tied to the image ID, never to a tag)
    if ! image_id="$(docker image inspect --format '{{.Id}}' "${image}" 2>/dev/null)" || [[ -z "${image_id}" ]]; then
        echo "no scan image '${image}' here (docker load it first)" >&2
        exit 2
    fi
    image_tag="${image}"
    image="${image_id}"  # run what was inspected: a tag moved in between never runs under the recorded ID
    # this run's id: the node stamps it into every v2 scan (patch 0002), the adapter drops any other run's. Its first
    # digit is a-f, so the ROS parameter parser never reads it as a number (an all-digit id would be an integer)
    run_id="$(od -An -N8 -tx1 /dev/urandom | tr -d ' \n')"
    run_id="$(printf '%x' $(( 10 + 0x${run_id:0:2} % 6 )))${run_id:1}"
    # the LiDAR's USB adapter as udev names it (vendor:product:serial; empty when udev cannot say) and this host
    usb_id=""
    if command -v udevadm >/dev/null 2>&1; then
        props="$(udevadm info --query=property --name="${device}" 2>/dev/null || true)"
        vid="$(sed -n 's/^ID_VENDOR_ID=//p' <<<"${props}")"
        pid="$(sed -n 's/^ID_MODEL_ID=//p' <<<"${props}")"
        serial="$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"${props}")"
        if [[ -n "${vid}" && -n "${pid}" && -n "${serial}" ]]; then
            usb_id="${vid}:${pid}:${serial}"  # no serial, no identity: adapters share vendor and product ids
        fi
    fi
    host_name="$(hostname)"
    echo "scan run ${run_id}: image ${image_tag} id ${image_id}, usb ${usb_id:-unknown}, host ${host_name}" >&2
    docker_args+=(
        --device "${device}:${device}"
        --group-add "$(stat -L -c %g "${device}")"  # the device's own group, through a by-id link
        --mount "type=bind,src=${sock_dir},dst=/run/forklift-scan"
    )
    driver="ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=${device} -p serial_baudrate:=256000"
    driver+=" -p frame_id:=laser -p angle_compensate:=true -p scan_mode:=Sensitivity -r scan:=/sensors/lidar/scan_raw"
    driver+=" -r scan_sdk:=/sensors/lidar/scan_raw_sdk"  # forklift patch 0001 (PR 7a-2)
    driver+=" -r scan_sdk_info:=/sensors/lidar/scan_sdk_info -p forklift_run_id:=${run_id}"  # patch 0002 (7a-3)
    # bench only (7a-2 timetable 1): hold each read scan this long before publishing it
    delay_ms="${FORKLIFT_SDK_PUBLISH_DELAY_MS:-0}"
    if ! [[ "${delay_ms}" =~ ^[0-9]{1,4}$ ]] || (( 10#${delay_ms} > 1000 )); then
        echo "FORKLIFT_SDK_PUBLISH_DELAY_MS must be 0-1000, not '${delay_ms}'" >&2
        exit 2
    fi
    delay_ms=$(( 10#${delay_ms} ))  # decimal, one value for the check, the argument and the log (0100 is 100)
    if (( delay_ms > 0 )); then
        driver+=" -p publish_delay_ms:=${delay_ms}"
        echo "bench: each scan held ${delay_ms} ms before it is published" >&2
    fi
    # bench only (7a-3 plan 4 (a)): patch 0002's receive-thread record, into a host folder of this run's own
    if [[ -n "${FORKLIFT_SDK_RX_LOG_DIR:-}" ]]; then
        mkdir -p "${FORKLIFT_SDK_RX_LOG_DIR}"
        docker_args+=(
            --mount "type=bind,src=$(cd -- "${FORKLIFT_SDK_RX_LOG_DIR}" && pwd -P),dst=/run/forklift-sdk-log"
            --env "FORKLIFT_SDK_RX_LOG=/run/forklift-sdk-log/sdk_rx_${run_id}.log"
        )
        echo "bench: SDK receive record ${FORKLIFT_SDK_RX_LOG_DIR}/sdk_rx_${run_id}.log" >&2
    fi
    # FORKLIFT_SCAN_SOURCE=sdk: the adapter reads the patched node's first-sample-stamped topic (the observe run and
    # the layer need it); raw (the default) keeps the teleop bridge's call-start stamps as they were
    adapter="python3 ros2/src/forklift_ros/forklift_ros/real_scan_adapter.py --socket /run/forklift-scan/scan.sock"
    source="${FORKLIFT_SCAN_SOURCE:-raw}"
    if [[ "${source}" != "raw" && "${source}" != "sdk" ]]; then
        echo "FORKLIFT_SCAN_SOURCE must be raw or sdk, not '${source}'" >&2
        exit 2
    fi
    adapter+=" --source ${source}"
    if [[ "${source}" == "sdk" ]]; then
        # the source facts every v2 frame carries (the host compares them with lidar.yaml timing.applies_to)
        adapter+=" --run-id ${run_id} --image-id $(printf '%q' "${image_id}") --host $(printf '%q' "${host_name}")"
        if [[ -n "${usb_id}" ]]; then
            adapter+=" --usb-id $(printf '%q' "${usb_id}")"
        fi
    fi
    # the driver in the background of this shell (sourced here first), the adapter in front; on exit the whole
    # process group goes (ros2 run's node is its child)
    set -- bash -c "source /opt/sllidar_ws/install/setup.bash || exit 1; trap 'kill 0' EXIT; ${driver} & ${adapter}"
fi

if [[ -n "${FORKLIFT_DOCKER_NAME:-}" ]]; then
    docker_args+=(--name "${FORKLIFT_DOCKER_NAME}")  # so a caller can stop the container by name (tools/hwday)
fi

if [[ -t 0 && -t 1 ]]; then
    docker_args+=(--tty)
fi

if (( $# == 0 )); then
    set -- bash
fi

exec docker "${docker_args[@]}" "${image}" "$@"
