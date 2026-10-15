#!/bin/bash
#SBATCH --job-name=console_replay
#SBATCH --partition=compute
#SBATCH --cpus-per-task=4
#SBATCH --mem=3G
#SBATCH --time=00:30:00
#SBATCH --output=slurm-console-replay-%j.log
# Monitor-only console on a recorded Isaac SLAM bag (C2a demo, CPU only).
#
#   sbatch deploy/console/replay_demo.sbatch <bag directory> [passes]
#
# Run from the repository root (it is mounted into the console image). The bag comes
# from `python3 -m forklift_ros.slam_replay` (/scan, /tf, /odom in sim time). Each pass
# starts slam_toolbox (online sync, the replay parameters) and plays the bag once with
# /clock; the gateway keeps running across passes, so the sim clock jumping back is part
# of the demo. The chosen port is written to console-replay-<job>.port; open it with
#   ssh -N -L 8765:127.0.0.1:<port> <host>   then   http://localhost:8765/
set -u
BAG=$(realpath "${1:?bag directory}")
PASSES=${2:-3}
ROOT=$(pwd)
PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
NAME=console-replay-$SLURM_JOB_ID
echo "$PORT" > "console-replay-$SLURM_JOB_ID.port"
echo "host=$(hostname) port=$PORT job=$SLURM_JOB_ID bag=$BAG passes=$PASSES"
trap 'docker stop -t 3 "$NAME" >/dev/null 2>&1' EXIT TERM INT
# --group-add 2000: recorded files belong to the lab group; a private ROS domain on
# localhost keeps this replay off other jobs' graphs.
docker run --rm --name "$NAME" --network host --group-add 2000 \
  -e ROS_DOMAIN_ID=$((SLURM_JOB_ID % 200 + 20)) -e ROS_LOCALHOST_ONLY=1 \
  -v "$ROOT":/workspace -v "$BAG":/data/bag:ro -w /workspace \
  forklift/ros2-console:jazzy bash -c "
    python3 -m console.gateway --backend isaac --vehicle-id isaac-replay \
      --host 127.0.0.1 --port $PORT --ros-args -p use_sim_time:=true \
      > console-replay-$SLURM_JOB_ID.gateway.log 2>&1 &
    for pass in \$(seq 1 $PASSES); do
      echo \"pass \$pass start \$(date +%T)\"
      ros2 launch slam_toolbox online_sync_launch.py \
        slam_params_file:=/workspace/ros2/src/forklift_bringup/config/slam_toolbox_isaac_replay.yaml \
        use_sim_time:=true autostart:=true > /dev/null 2>&1 &
      SLAM=\$!
      sleep 4
      ros2 bag play /data/bag --clock 100 --rate 1.0 --topics /scan /tf /odom > /dev/null 2>&1
      sleep 15   # the last map lands; then the pose goes stale on screen
      kill \$SLAM; wait \$SLAM 2>/dev/null
      echo \"pass \$pass end \$(date +%T)\"
    done
    sleep 30
  "
