# Console gateway image (design C2a, D10): the ROS 2 development image plus websockets.
# The base image stays untouched so the SLAM replay batches keep their environment.
#   docker build -t forklift/ros2-console:jazzy -f deploy/console/Dockerfile deploy/console
ARG BASE_IMAGE="forklift/ros2-dev:jazzy"
FROM ${BASE_IMAGE}

RUN /opt/forklift/venv/bin/python -m pip install --no-cache-dir "websockets==16.0"
