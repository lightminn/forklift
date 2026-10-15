#!/bin/bash
#SBATCH --job-name=console_replay
#SBATCH --partition=compute
#SBATCH --cpus-per-task=4
#SBATCH --mem=3G
#SBATCH --time=00:30:00
#SBATCH --output=slurm-console-replay-%j.log
#SBATCH --signal=B:TERM@60
# Monitor-only console on a recorded Isaac SLAM bag (C2a demo, CPU only).
#
#   sbatch deploy/console/replay_demo.sbatch <bag directory> [passes]
#
# Run from the repository root (it is mounted into the console image). The bag comes
# from `python3 -m forklift_ros.slam_replay` (/scan, /tf, /odom in sim time). Each pass
# starts slam_toolbox (online sync, the replay parameters) and plays the bag once with
# /clock; the gateway keeps running across passes, so the sim clock jumping back is part
# of the demo. Once the gateway answers, its port is written to
# console-replay-<job>.port; open it with
#   ssh -N -L 8765:127.0.0.1:<port> <host>   then   http://localhost:8765/
set -u
BAG=$(realpath "${1:?bag directory}")
PASSES=${2:-3}
if ! [[ $PASSES =~ ^[1-9][0-9]?$ ]]; then
  echo "passes must be an integer from 1 to 99, got '$PASSES'" >&2
  exit 2
fi
ROOT=$(pwd)
JOB=${SLURM_JOB_ID:?run under Slurm}
NAME=console-replay-$JOB
PORTFILE=console-replay-$JOB.port
rm -f "$PORTFILE"
# A free port for the host side of the published port. If another process takes it in
# the meantime, `docker run -p` fails and so does this job -- nothing runs half-started.
PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
echo "host=$(hostname) job=$JOB bag=$BAG passes=$PASSES"

# The container runs detached and the script waits with the interruptible `wait`
# builtin: a foreground `docker run` delays the trap until it returns, and Slurm's kill
# of the client left the container running (ws1, 2026-10-15). --signal sends TERM 60 s
# before the time limit so a timeout cleans up the same way as scancel.
cleanup() { docker stop -t 3 "$NAME" >/dev/null 2>&1; rm -f "console-replay-$JOB.exit"; }
trap cleanup EXIT
trap 'cleanup; exit 143' TERM INT

# Own network namespace (bridge) + discovery on localhost only: the replay's ROS graph
# (/clock, TF, scans) cannot mix with any other job's, whatever its domain id.
# --group-add 2000: recorded files belong to the lab group.
docker run -d --rm --name "$NAME" -p "127.0.0.1:$PORT:8765" --group-add 2000 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$ROOT":/workspace -v "$BAG":/data/bag:ro -w /workspace \
  forklift/ros2-console:jazzy bash -c "
    python3 -m console.gateway --backend isaac --vehicle-id isaac-replay \
      --host 0.0.0.0 --port 8765 --ros-args -p use_sim_time:=true \
      > console-replay-$JOB.gateway.log 2>&1 &
    GW=\$!
    for pass in \$(seq 1 $PASSES); do
      kill -0 \$GW 2>/dev/null || { echo 'gateway died'; exit 3; }
      echo \"pass \$pass start \$(date +%T), slam nodes running: \$(pgrep -fc '[s]ync_slam_toolbox_node')\"
      ros2 launch slam_toolbox online_sync_launch.py \
        slam_params_file:=/workspace/ros2/src/forklift_bringup/config/slam_toolbox_isaac_replay.yaml \
        use_sim_time:=true autostart:=true > /dev/null 2>&1 &
      SLAM=\$!
      sleep 4
      kill -0 \$SLAM 2>/dev/null || { echo 'slam_toolbox did not start'; exit 4; }
      ros2 bag play /data/bag --clock 100 --rate 1.0 --topics /scan /tf /odom > /dev/null 2>&1 \
        || { echo 'bag play failed'; kill \$SLAM; exit 5; }
      sleep 15   # the last map lands; then the pose goes stale on screen
      kill -0 \$SLAM 2>/dev/null || { echo 'slam_toolbox died during the pass'; exit 4; }
      # SIGINT is ros2 launch's orderly shutdown (SIGTERM may leave its children); the
      # next pass must not share map or TF with this one. [s] keeps this shell's own
      # command line from matching.
      kill -INT \$SLAM
      for _ in \$(seq 1 20); do kill -0 \$SLAM 2>/dev/null || break; sleep 0.5; done
      pkill -INT -f '[s]ync_slam_toolbox_node'; sleep 2
      pkill -KILL -f '[s]ync_slam_toolbox_node'
      wait \$SLAM 2>/dev/null
      if pgrep -f '[s]ync_slam_toolbox_node' > /dev/null; then echo 'slam_toolbox left running'; exit 4; fi
      echo \"pass \$pass end \$(date +%T)\"
    done
    sleep 30
    kill -0 \$GW 2>/dev/null || { echo 'gateway died'; exit 3; }
  " > /dev/null || { echo "container did not start (port $PORT taken?)" >&2; exit 1; }

# Publish the port only once the gateway answers (60 s in all, 2 s per request).
deadline=$((SECONDS + 60))
while [ $SECONDS -lt $deadline ]; do
  if curl -fs --max-time 2 -o /dev/null "http://127.0.0.1:$PORT/"; then
    echo "$PORT" > "$PORTFILE"
    echo "gateway ready: port $PORT"
    break
  fi
  sleep 1
done
[ -s "$PORTFILE" ] || { echo "gateway did not answer within 60 s" >&2; docker logs "$NAME" >&2; exit 1; }

docker logs -f "$NAME" &   # pass lines into this log; ends with the container
docker wait "$NAME" > "console-replay-$JOB.exit" &
wait $!
code=$(cat "console-replay-$JOB.exit" 2>/dev/null || echo 1)
rm -f "console-replay-$JOB.exit"
exit "$code"
