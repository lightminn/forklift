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
    && rm -rf /var/lib/apt/lists/*

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

# Install a non-editable core snapshot in an isolated venv. During development,
# Python commands started in /workspace resolve the bind-mounted checkout first.
COPY pyproject.toml /opt/forklift/source/pyproject.toml
COPY forklift_core /opt/forklift/source/forklift_core
RUN python3 -m venv --system-site-packages /opt/forklift/venv \
    && /opt/forklift/venv/bin/python -m pip install \
        --no-build-isolation --no-deps /opt/forklift/source \
    && rm -rf /opt/forklift/source

ENV PATH="/opt/forklift/venv/bin:${PATH}" \
    PYTHONUNBUFFERED=1

WORKDIR /workspace
USER forklift-dev

# The inherited /ros_entrypoint.sh is a Bash entrypoint that sources Jazzy's
# setup.bash before executing the command array.
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
