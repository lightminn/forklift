ARG ROS_BASE_IMAGE="ros:jazzy-ros-base-noble@sha256:386d06ec6d4188f731bae5678e07b4cb64a4e4d4152090c0bd1f881dcf7706f5"
FROM ${ROS_BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-rosbag2-py \
        ros-jazzy-rosbag2-storage-default-plugins \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-msgs \
        ros-jazzy-sensor-msgs \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pil \
        python3-yaml \
        python3-pytest \
        python3-setuptools \
        libegl1-mesa-dev \
        libgl1-mesa-dri \
        mesa-utils \
        ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Source is supplied as a read-only runtime snapshot. Build/install outputs and
# application caches belong in the per-run output, never the source mount.
ENV PYTHONUNBUFFERED=1 \
    ROS_LOCALHOST_ONLY=1 \
    OMP_NUM_THREADS=2 \
    OPENBLAS_NUM_THREADS=2 \
    LP_NUM_THREADS=2
WORKDIR /workspace
USER 1000:1000
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
