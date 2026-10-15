"""Run the monitor-only gateway (C2a) inside the ROS 2 container.

    python -m console.gateway --backend isaac --vehicle-id fl-1 --port 8765 \
        [--ros-args -p use_sim_time:=true -p camera_topic:=/camera/color/image_raw/compressed]

Everything after the gateway's own options is handed to rclpy.
"""

import argparse
import asyncio
import secrets
from pathlib import Path

from console.gateway import server
from console.gateway.convert import ISAAC_STOP_INSTRUCTION
from console.gateway.store import Store

WEB_ROOT = Path(__file__).resolve().parent.parent / "web"
# dls08_measured rear_axle_x_m (base_link frame, an estimate kept fixed)
REAR_AXLE_X_M = -0.34


async def _serve(store: Store, args, capture_domain: str) -> None:
    gw = await server.start(
        store,
        args.host,
        args.port,
        Path(args.web),
        args.vehicle_id,
        args.backend,
        secrets.randbits(48) | 1,  # a new session for every start (contract 2)
        args.rear_axle_x,
        capture_domain=capture_domain,
        stop_instruction=args.stop_instruction,
    )
    print(
        f"console gateway (monitor only): http://{args.host}:{args.port}/ "
        f"backend={args.backend} epoch={gw.epoch}",
        flush=True,
    )
    try:
        await asyncio.Event().wait()
    finally:
        gw.close()
        await gw.wait_closed()


def parse_args(argv=None):
    """(gateway options, the rest for rclpy). A real truck needs --stop-instruction."""
    p = argparse.ArgumentParser(prog="python -m console.gateway")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--web", default=str(WEB_ROOT))
    p.add_argument("--vehicle-id", default="forklift-1")
    p.add_argument("--backend", choices=("real", "isaac"), required=True)
    p.add_argument("--rear-axle-x", type=float, default=REAR_AXLE_X_M)
    p.add_argument(
        "--stop-instruction",
        help="what the screen tells the operator to do instead of STOP "
        "(required for --backend real: the stop method that really exists on it)",
    )
    args, ros_args = p.parse_known_args(argv)
    if args.stop_instruction is None and args.backend == "isaac":
        args.stop_instruction = ISAAC_STOP_INSTRUCTION
    if not (args.stop_instruction or "").strip():
        p.error("--stop-instruction is required for --backend real")
    return args, ros_args


def main(argv=None) -> None:
    args, ros_args = parse_args(argv)

    import rclpy

    from console.gateway.ros_node import spin_in_thread

    store = Store()
    node, _ = spin_in_thread(store, args=["console_gateway", *ros_args])
    try:
        asyncio.run(_serve(store, args, "sim" if node.sim_time else "sensor"))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
