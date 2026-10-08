# RTAB-Map (visual / visual+LiDAR SLAM) on top of the project's ROS 2 Jazzy
# development image. Built FROM the existing image so slam_toolbox and the rest
# of the LiDAR baseline stay the exact binaries the earlier records used.
# Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md
ARG BASE_IMAGE="forklift/ros2-dev:jazzy"
FROM ${BASE_IMAGE}

USER root
RUN apt-get update \
    && apt-get install --yes --no-install-recommends --no-upgrade \
        ros-jazzy-rtabmap-ros \
        ros-jazzy-robot-localization \
    && rm -rf /var/lib/apt/lists/*
USER forklift-dev
