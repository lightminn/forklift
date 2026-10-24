#!/usr/bin/env python3
"""Does Isaac Sim reproduce the same RGB image bit for bit across processes? (deterministic-run plan, step 1)

``determinism_probe.py`` answered the question for depth on a static box (bit-identical). Two same-seed transport
runs (2026-10-22/23, x1_trailer_slam vs bs91_trailer_s0) had bit-identical depth and instance ids for frames of the
same stamp but RGB differing in about half the pixels (up to 75 levels), which moved the learned proposals and, through
them, the pocket estimate. This probe asks what makes RGB reproducible:

  * a camera moved along a fixed path, one ``world.step(render=True)`` per frame -- the render cadence is the same in
    every process, so a difference left is the renderer's own, not a different frame history;
  * optional ``--base-scene`` (the transport runs' scene, its lights and materials) instead of a ground and a box;
  * ``--carb KEY=VALUE`` settings written after the app starts and read back (recorded), to try the anti-aliasing,
    jitter and temporal-denoiser keys one at a time.

Each frame's RGB(A) and depth are hashed (SHA-256 of the raw arrays); two processes are compared by hash alone.
Writes only into ``--out``.

    $ISAAC_PY sim/isaac/rgb_determinism_probe.py --out OUT --tag a [--base-scene scene.usda] [--carb /rtx/post/aa/op=0]
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path


def parse_carb(items):
    """KEY=VALUE strings to (key, value) with ints, floats and booleans parsed."""
    out = []
    for item in items:
        key, _, raw = item.partition("=")
        if not key.startswith("/") or not raw:
            raise ValueError(f"--carb wants /key=value, got {item!r}")
        low = raw.lower()
        if low in ("true", "false"):
            value = low == "true"
        else:
            try:
                value = int(raw)
            except ValueError:
                try:
                    value = float(raw)
                except ValueError:
                    value = raw
        out.append((key, value))
    return out


def camera_path(frame: int):
    """A fixed path: a slow arc towards and across the origin (positions in m, the camera looking along +x)."""
    s = frame / 60.0
    return (-3.0 + 0.5 * s, 0.3 * s, 0.6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--base-scene", type=Path, default=None)
    ap.add_argument("--warmup", type=int, default=30)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--resolution", type=int, nargs=2, default=(640, 480))
    ap.add_argument("--carb", action="append", default=[], help="/key=value, written after start and read back")
    ap.add_argument("--anti-aliasing", type=int, default=None,
                    help="SimulationApp anti_aliasing (0 off, 1 TAA, 2 FXAA, 3 DLSS -- Isaac's default, 4 RTXAA)")
    ap.add_argument("--sync-frame", action="store_true",
                    help="after each step, render (no physics) until the frame's rendering_time is the step's time")
    ap.add_argument("--save-rgb", type=int, default=0, help="keep the first N frames' RGB in <tag>_rgb.npz")
    ap.add_argument("--extra-arg", action="append", default=[],
                    help="a Kit argument at launch (SimulationApp extra_args), e.g. --/rtx/...=value")
    ap.add_argument("--vary-load", type=int, default=0,
                    help="0..N extra app updates before the world (N from the process id): a setup whose length "
                    "differs between processes, as a runner's loading does")
    ap.add_argument("--second-camera", action="store_true", help="a 1280x720 camera rendering too (the video's)")
    ap.add_argument("--render-every", type=int, default=1, help="render on every k-th step only (the others not)")
    ap.add_argument("--pollute", type=int, default=0,
                    help="0..N rendered steps (N from the process id) with the camera elsewhere before the warm-up: a "
                    "render history that differs between processes, as a runner's setup renders do")
    ap.add_argument("--static", action="store_true", help="the camera holds still through the warm-up and the frames")
    ap.add_argument("--direct-read", action="store_true",
                    help="also hash the direct getters (get_rgba / get_depth) after the sync, as the runner reads pixels")
    ap.add_argument("--carb-after", action="append", default=[],
                    help="/key=value written after the cameras are initialized and read back")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    carb_items = parse_carb(args.carb)
    width, height = args.resolution

    from isaacsim import SimulationApp  # before any other Isaac import

    launch = {"headless": True, "width": width, "height": height, "renderer": "RaytracedLighting", "multi_gpu": False}
    if args.anti_aliasing is not None:
        launch["anti_aliasing"] = int(args.anti_aliasing)
    if args.extra_arg:
        launch["extra_args"] = list(args.extra_arg)
    app = SimulationApp(launch)
    record = {"tag": args.tag, "executable": sys.executable,
              "config": {"base_scene": str(args.base_scene) if args.base_scene else None, "warmup": args.warmup,
                         "frames": args.frames, "resolution": [width, height], "anti_aliasing": args.anti_aliasing,
                         "sync_frame": args.sync_frame, "extra_args": args.extra_arg, "vary_load": args.vary_load,
                         "second_camera": args.second_camera, "render_every": args.render_every},
              "carb": {}, "frames": [], "errors": []}
    try:
        import carb
        import numpy as np
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import FixedCuboid, GroundPlane
        from isaacsim.sensors.camera import Camera

        settings = carb.settings.get_settings()
        for key, value in carb_items:
            settings.set(key, value)
        watch = ["/rtx/post/aa/op", "/rtx/post/taa/samples", "/rtx/post/dlss/execMode", "/rtx/rendermode",
                 "/rtx/directLighting/sampledLighting/enabled", "/rtx/ambientOcclusion/enabled",
                 "/rtx/reflections/enabled", "/rtx/indirectDiffuse/enabled"] + [k for k, _ in carb_items]
        if args.base_scene is not None:
            omni.usd.get_context().open_stage(str(args.base_scene))
            for _ in range(20):
                app.update()
        import os

        extra_updates = (os.getpid() * 7919) % (args.vary_load + 1) if args.vary_load else 0
        record["extra_updates"] = extra_updates
        for _ in range(extra_updates):
            app.update()
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 120.0, rendering_dt=1 / 120.0)
        if args.base_scene is None:
            world.scene.add(GroundPlane(prim_path="/World/ground", size=20.0))
            world.scene.add(FixedCuboid(prim_path="/World/box", name="box", position=np.array([3.0, 0.0, 0.25]),
                                        scale=np.array([0.66, 0.66, 0.09])))
        cam = Camera(prim_path="/World/probe_cam", frequency=-1, resolution=(width, height))
        cam2 = Camera(prim_path="/World/probe_cam2", frequency=-1, resolution=(1280, 720)) if args.second_camera else None
        world.reset()
        cam.initialize()
        if cam2 is not None:
            cam2.initialize()
            cam2.set_world_pose(position=np.array([0.0, 0.0, 9.0]), orientation=np.array([0.0, 0.7071068, 0.0, 0.7071068]),
                                camera_axes="usd")
        cam.add_distance_to_image_plane_to_frame()

        after_items = parse_carb(args.carb_after)
        for key, value in after_items:
            settings.set(key, value)
        watch += [k for k, _ in after_items]

        def place(frame):
            if args.static:
                frame = 0
            cam.set_world_pose(position=np.array(camera_path(frame)), orientation=np.array([0.5, -0.5, 0.5, 0.5]),
                               camera_axes="usd")

        polluted = (os.getpid() * 104729) % (args.pollute + 1) if args.pollute else 0
        record["polluted_steps"] = polluted
        for f in range(polluted):
            cam.set_world_pose(position=np.array([3.0 * np.sin(f), 3.0 * np.cos(f), 1.5]),
                               orientation=np.array([0.5, -0.5, 0.5, 0.5]), camera_axes="usd")
            world.step(render=True)

        for f in range(args.warmup):
            place(f)
            world.step(render=True)
        record["carb"] = {k: settings.get(k) for k in watch}
        kept = {}
        for i in range(args.frames):
            f = args.warmup + i
            for k in range(args.render_every - 1):
                place(f)
                world.step(render=False)
            place(f)
            world.step(render=True)
            extra = 0
            if args.sync_frame:
                while extra < 20 and abs(float(cam.get_current_frame().get("rendering_time") or -1.0)
                                         - float(world.current_time)) > 1e-6:
                    world.render()
                    extra += 1
            frame = cam.get_current_frame()
            rgba, depth = frame.get("rgba"), frame.get("distance_to_image_plane")
            entry = {"frame": f, "rendering_time": frame.get("rendering_time"), "sim_time": float(world.current_time),
                     "extra_renders": extra}
            if i < args.save_rgb and rgba is not None and len(rgba):
                kept[f"f{f}"] = np.asarray(rgba)[..., :3].astype(np.uint8)
            for name, arr in (("rgb", rgba), ("depth", depth)):
                if arr is None or not hasattr(arr, "__len__") or len(arr) == 0:
                    entry[name] = None
                else:
                    a = np.ascontiguousarray(arr)
                    entry[name] = hashlib.sha256(a.tobytes()).hexdigest()[:16]
                    entry[f"{name}_sum"] = float(np.asarray(a, dtype=np.float64)[np.isfinite(a)].sum())
            if args.direct_read:
                for name, arr in (("rgb_direct", cam.get_rgba()), ("depth_direct", cam.get_depth())):
                    entry[name] = (hashlib.sha256(np.ascontiguousarray(arr).tobytes()).hexdigest()[:16]
                                   if arr is not None and hasattr(arr, "__len__") and len(arr) else None)
            record["frames"].append(entry)
        if kept:
            np.savez_compressed(args.out / f"{args.tag}_rgb.npz", **kept)
    except BaseException:
        import traceback

        record["errors"].append(traceback.format_exc())
    finally:
        (args.out / f"{args.tag}.json").write_text(json.dumps(record, indent=2) + "\n")
        print("PROBE_DONE", args.tag, len(record["frames"]), "frames", len(record["errors"]), "errors", flush=True)
        app.close()


if __name__ == "__main__":
    main()
