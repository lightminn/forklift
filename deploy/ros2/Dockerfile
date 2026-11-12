ARG ROS_BASE_IMAGE="ros:jazzy-ros-base-noble@sha256:386d06ec6d4188f731bae5678e07b4cb64a4e4d4152090c0bd1f881dcf7706f5"
FROM ${ROS_BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG DEV_UID=1000
ARG DEV_GID=1000

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pip \
        python3-pytest \
        python3-rosdep \
        python3-setuptools \
        python3-venv \
        python3-wheel \
        ros-jazzy-demo-nodes-cpp \
        ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# The real A2M12 driver (bring-up 7a, docs/design/2026-11-12-real-scan-obstacle-layer.md 1.1), pinned by commit.
# Its stamp is taken before it fetches a scan (not the acquisition time); the design records what that means.
ARG SLLIDAR_ROS2_COMMIT=34300099fadfc772965962dec837bf436706188f
RUN git clone https://github.com/Slamtec/sllidar_ros2.git /opt/sllidar_ws/src/sllidar_ros2 \
    && git -C /opt/sllidar_ws/src/sllidar_ros2 checkout --detach "${SLLIDAR_ROS2_COMMIT}" \
    && rm -rf /opt/sllidar_ws/src/sllidar_ros2/.git \
    && source /opt/ros/jazzy/setup.bash \
    && cd /opt/sllidar_ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/sllidar_ws/build /opt/sllidar_ws/log

# Noble's ROS base image already has an ubuntu account at UID/GID 1000. Reuse
# that numeric identity when it matches, and create an account for other host IDs.
RUN set -eux; \
    if [[ "${DEV_UID}" == "0" || "${DEV_GID}" == "0" ]]; then \
        echo "DEV_UID and DEV_GID must identify a non-root host user" >&2; \
        exit 2; \
    fi; \
    dev_group="$(getent group "${DEV_GID}" | cut -d: -f1 || true)"; \
    if [[ -z "${dev_group}" ]]; then \
        groupadd --gid "${DEV_GID}" forklift-dev; \
        dev_group="forklift-dev"; \
    fi; \
    dev_user="$(getent passwd "${DEV_UID}" | cut -d: -f1 || true)"; \
    if [[ -n "${dev_user}" && "${dev_user}" != "forklift-dev" ]]; then \
        usermod --login forklift-dev --home /home/forklift-dev --move-home "${dev_user}"; \
    elif [[ -z "${dev_user}" ]]; then \
        useradd --create-home --uid "${DEV_UID}" --gid "${DEV_GID}" \
            --shell /bin/bash forklift-dev; \
    fi; \
    usermod --gid "${DEV_GID}" forklift-dev; \
    install -d -o "${DEV_UID}" -g "${DEV_GID}" /workspace

# Editable install that points at /workspace/src. The launcher bind-mounts the
# checkout at /workspace, so imports resolve to the live source from any cwd.
# Without the mount, the copy baked below is used (stale snapshot semantics).
COPY pyproject.toml /workspace/pyproject.toml
COPY src /workspace/src
RUN python3 -m venv --system-site-packages /opt/forklift/venv \
    && /opt/forklift/venv/bin/python -m pip install \
        --no-build-isolation --no-deps -e /workspace \
    && chown -R "${DEV_UID}:${DEV_GID}" /workspace /opt/forklift/venv

ENV PATH="/opt/forklift/venv/bin:${PATH}" \
    PYTHONUNBUFFERED=1

WORKDIR /workspace
USER forklift-dev

# The inherited /ros_entrypoint.sh is a Bash entrypoint that sources Jazzy's
# setup.bash before executing the command array.
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
