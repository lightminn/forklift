"""Construct a provisional, photo-informed DLS08 assembly in metres.

The catalogue identifies an externally matching candidate, not the delivered SKU.
Decorative proportions, mass distribution and actuation remain approximations.
"""

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

YELLOW = (1.0, 0.72, 0.015, 1.0)
BLACK = (0.045, 0.052, 0.065, 1.0)
RUBBER = (0.023, 0.026, 0.032, 1.0)
GREY = (0.25, 0.29, 0.34, 1.0)
WHITE = (0.88, 0.93, 1.0, 1.0)
RED = (0.75, 0.035, 0.025, 1.0)


@dataclass
class Part:
    name: str
    shape: str
    size: tuple
    pos: tuple
    color: tuple
    rpy: tuple = (0.0, 0.0, 0.0)
    collision: bool = True


@dataclass
class Link:
    name: str
    parent: str | None
    pos: tuple
    mass_kg: float
    inertia_size: tuple
    center_of_mass: tuple = (0.0, 0.0, 0.0)
    joint_name: str | None = None
    joint_type: str = "fixed"
    axis: tuple = (0.0, 0.0, 1.0)
    limits: tuple | None = None
    parts: list = field(default_factory=list)
    sites: dict = field(default_factory=dict)


def load_parameters(path: Path) -> dict:
    """Reject missing/unknown fields and impossible geometry before writing files."""
    try:
        value = yaml.safe_load(path.read_text())
    except yaml.YAMLError as error:
        raise ValueError("parameters: invalid YAML") from error
    keys = {
        "catalogue": "overall_length_m overall_width_m overall_height_m net_mass_kg",
        "dimensions": (
            "rear_extent_x_m front_axle_x_m rear_axle_x_m wheel_radius_m "
            "wheel_width_m wheel_track_m body_front_x_m mast_x_m mast_height_m "
            "mast_width_m fork_root_x_m fork_spacing_m fork_width_m "
            "fork_thickness_m fork_center_height_m carriage_width_m "
            "carriage_height_m lift_travel_m"
        ),
        "assumptions": (
            "steering_axle steering_limit_rad steering_torque_limit_nm "
            "urdf_steering_speed_limit_radps urdf_lift_speed_limit_mps wheel_mass_kg "
            "steering_carrier_mass_kg carriage_mass_kg sliding_friction "
            "wheel_speed_limit_radps wheel_torque_limit_nm lift_force_limit_n "
            "lift_position_gain"
        ),
        "evidence": "identity catalogue dimensions dynamics",
    }
    if not isinstance(value, dict) or set(value) != {*keys, "model_name"}:
        raise ValueError("parameters: unexpected or missing top-level fields")
    if value["model_name"] != "dls08_provisional":
        raise ValueError("parameters: model_name must preserve provisional identity")
    for group, names in keys.items():
        if not isinstance(value[group], dict) or set(value[group]) != set(
            names.split()
        ):
            raise ValueError(f"parameters: unexpected or missing {group} fields")
        for name, number in value[group].items():
            if group == "evidence" or name == "steering_axle":
                if not isinstance(number, str) or not number:
                    raise ValueError(f"parameters: {group}.{name} must be text")
                continue
            if (
                isinstance(number, bool)
                or not isinstance(number, (int, float))
                or not math.isfinite(number)
            ):
                raise ValueError(f"parameters: {group}.{name} must be finite")
            if not name.endswith("_x_m") and number <= 0:
                raise ValueError(f"parameters: {group}.{name} must be positive")
    d, c, a = value["dimensions"], value["catalogue"], value["assumptions"]
    if a["steering_axle"] not in {"front", "rear"}:
        raise ValueError("parameters: steering_axle must be front or rear")
    tip = d["rear_extent_x_m"] + c["overall_length_m"]
    if not (
        d["rear_extent_x_m"]
        < d["rear_axle_x_m"]
        < 0
        < d["front_axle_x_m"]
        < d["body_front_x_m"]
        < d["mast_x_m"]
        < d["fork_root_x_m"]
        < tip
    ):
        raise ValueError("parameters: longitudinal geometry is inconsistent")
    if not (
        d["fork_width_m"] < d["fork_spacing_m"]
        and d["fork_spacing_m"] + d["fork_width_m"]
        < d["carriage_width_m"]
        < c["overall_width_m"]
    ):
        raise ValueError("parameters: fork gap or carriage width is invalid")
    if d["wheel_track_m"] + d["wheel_width_m"] > c["overall_width_m"]:
        raise ValueError("parameters: wheels extend beyond catalogue width")
    if (
        d["fork_center_height_m"] <= d["fork_thickness_m"] / 2
        or c["overall_height_m"] <= 0.326
        or d["mast_height_m"] >= c["overall_height_m"]
        or d["lift_travel_m"] + d["carriage_height_m"] > c["overall_height_m"]
    ):
        raise ValueError("parameters: vertical geometry is inconsistent")
    remaining = c["net_mass_kg"] - 4 * a["wheel_mass_kg"]
    remaining -= 2 * a["steering_carrier_mass_kg"] + a["carriage_mass_kg"]
    if remaining <= 0:
        raise ValueError("parameters: component masses exceed total mass")
    if a["steering_limit_rad"] >= math.pi / 2:
        raise ValueError("parameters: steering limit must be below pi/2")
    return value


def validate_assembly_envelope(links: list[Link], parameters: dict) -> None:
    """Check the complete neutral-pose visual bounds before writing any output.

    Link origins have no initial rotation. Articulated poses may extend beyond
    the catalogue envelope; this check covers lowered forks and straight wheels.
    """
    lower, upper = [math.inf] * 3, [-math.inf] * 3
    origins = {}
    for link in links:
        parent = origins[link.parent] if link.parent is not None else (0, 0, 0)
        origins[link.name] = tuple(parent[i] + link.pos[i] for i in range(3))
        for part in link.parts:
            if any(not math.isfinite(v) or v <= 0 for v in part.size):
                raise ValueError(f"parameters: invalid size for {part.name}")
            roll, pitch, yaw = part.rpy
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            rotation = (
                (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                (-sp, cp * sr, cp * cr),
            )
            for axis, row in enumerate(rotation):
                if part.shape == "box":
                    extent = sum(abs(row[j]) * part.size[j] / 2 for j in range(3))
                elif part.shape == "cylinder":
                    extent = part.size[0] * math.hypot(row[0], row[1])
                    extent += part.size[1] / 2 * abs(row[2])
                else:
                    raise ValueError(f"parameters: unsupported shape {part.shape}")
                center = origins[link.name][axis] + part.pos[axis]
                lower[axis] = min(lower[axis], center - extent)
                upper[axis] = max(upper[axis], center + extent)
    c = parameters["catalogue"]
    expected_lower = (
        parameters["dimensions"]["rear_extent_x_m"],
        -c["overall_width_m"] / 2,
        0.0,
    )
    sizes = (c["overall_length_m"], c["overall_width_m"], c["overall_height_m"])
    for axis in range(3):
        if not (
            math.isclose(lower[axis], expected_lower[axis], abs_tol=1e-6)
            and math.isclose(upper[axis] - lower[axis], sizes[axis], abs_tol=1e-6)
        ):
            raise ValueError(
                "parameters: generated neutral-pose envelope does not match "
                f"catalogue on axis {axis}; update related dimensions together"
            )


def assembly(parameters: dict) -> list[Link]:
    """Return one shared part/joint tree for the URDF and MJCF exporters."""
    d, c, a = (parameters[key] for key in ["dimensions", "catalogue", "assumptions"])
    width, height = c["overall_width_m"], c["overall_height_m"]
    rear, front = d["rear_extent_x_m"], d["body_front_x_m"]
    mass = c["net_mass_kg"] - 4 * a["wheel_mass_kg"]
    mass -= a["carriage_mass_kg"] + 2 * a["steering_carrier_mass_kg"]
    base = Link(
        "base_link",
        None,
        (0, 0, 0),
        mass,
        (front - rear, width * 0.8, 0.45),
        (-0.09, 0, 0.28),
    )
    links = [base]

    def box(link, name, size, pos, color=YELLOW, rpy=(0, 0, 0), collision=True):
        link.parts.append(
            Part(name, "box", tuple(size), tuple(pos), color, tuple(rpy), collision)
        )

    def cylinder(
        link, name, radius, length, pos, color, rpy=(0, 0, 0), collision=False
    ):
        link.parts.append(
            Part(
                name,
                "cylinder",
                (radius, length),
                tuple(pos),
                color,
                tuple(rpy),
                collision,
            )
        )

    def beam(link, name, start, end, thickness, color=BLACK, collision=True):
        delta = [end[i] - start[i] for i in range(3)]
        length = math.sqrt(sum(component**2 for component in delta))
        yaw = math.atan2(delta[1], delta[0])
        pitch = -math.atan2(delta[2], math.hypot(delta[0], delta[1]))
        box(
            link,
            name,
            (length, thickness, thickness),
            tuple((start[i] + end[i]) / 2 for i in range(3)),
            color,
            (0, pitch, yaw),
            collision,
        )

    # Narrow central body leaves room for wheels; fenders set the outer width.
    box(
        base,
        "underbody",
        (front - rear, width * 0.68, 0.105),
        ((front + rear) / 2, 0, 0.1875),
        BLACK,
    )
    box(base, "footwell", (0.50, width * 0.78, 0.05), (0.01, 0, 0.26))
    box(
        base,
        "rear_counterweight",
        (0.234, width * 0.84, 0.235),
        (rear + 0.123, 0, 0.3225),
    )
    box(base, "rear_hood", (0.30, width * 0.90, 0.055), (rear + 0.15, 0, 0.45))
    box(
        base,
        "rear_grille",
        (0.009, width * 0.57, 0.06),
        (rear + 0.0045, 0, 0.382),
        BLACK,
        collision=False,
    )
    for y in [-width * 0.32, width * 0.32]:
        box(
            base,
            f"rear_light_{y}",
            (0.012, 0.04, 0.025),
            (rear + 0.006, y, 0.40),
            RED,
            collision=False,
        )
    box(base, "dashboard_body", (0.16, width * 0.73, 0.15), (0.32, 0, 0.39))
    box(
        base,
        "dashboard_top",
        (0.19, width * 0.73, 0.035),
        (0.305, 0, 0.482),
        BLACK,
        (0, -0.2, 0),
        False,
    )
    box(
        base,
        "seat_cushion",
        (0.29, width * 0.61, 0.055),
        (-0.17, 0, 0.36),
        BLACK,
        collision=False,
    )
    box(
        base,
        "seat_back",
        (0.055, width * 0.63, 0.235),
        (-0.32, 0, 0.48),
        BLACK,
        (0, -0.12, 0),
        False,
    )
    box(
        base,
        "seat_insert",
        (0.009, width * 0.32, 0.16),
        (-0.279, 0, 0.49),
        YELLOW,
        (0, -0.12, 0),
        False,
    )

    for side, sign in [("left", 1), ("right", -1)]:
        y = sign * (width / 2 - 0.025)
        # Side doors remain closed; open frames preserve their characteristic holes.
        beam(base, f"{side}_door_top", (-0.32, y, 0.55), (0.24, y, 0.55), 0.043)
        beam(base, f"{side}_door_rear", (-0.32, y, 0.55), (-0.22, y, 0.33), 0.045)
        beam(base, f"{side}_door_bottom", (-0.22, y, 0.33), (0.10, y, 0.33), 0.045)
        beam(base, f"{side}_door_front", (0.10, y, 0.33), (0.24, y, 0.55), 0.048)
        beam(base, f"{side}_door_brace", (0.06, y, 0.36), (-0.06, y, 0.53), 0.035)
        box(base, f"{side}_sill", (0.32, 0.05, 0.06), (-0.01, y, 0.23))
        # Cabin posts and roof slats are distinct collision shapes, not a solid cab.
        beam(
            base,
            f"{side}_rear_post",
            (-0.39, y, 0.43),
            (-0.39, y, height - 0.036),
            0.045,
        )
        beam(
            base,
            f"{side}_front_post",
            (0.36, y, 0.29),
            (0.17, y, height - 0.036),
            0.045,
        )
        beam(
            base,
            f"{side}_roof_rail",
            (-0.41, y, height - 0.032),
            (0.17, y, height - 0.032),
            0.04,
        )
        beam(
            base,
            f"{side}_grab_handle",
            (0.29, y, 0.65),
            (0.245, y, 0.82),
            0.016,
            collision=False,
        )
        # Mount the lamp ahead of the sloped post instead of burying its lens.
        lamp_z = height - 0.145
        post_x = 0.36 - 0.19 * (lamp_z - 0.29) / (height - 0.036 - 0.29)
        box(
            base,
            f"{side}_front_lamp",
            (0.026, 0.045, 0.065),
            (post_x + 0.032, y, lamp_z),
            GREY,
            collision=False,
        )
        box(
            base,
            f"{side}_lamp_red",
            (0.004, 0.032, 0.03),
            (post_x + 0.047, y, lamp_z + 0.016),
            RED,
            collision=False,
        )
        box(
            base,
            f"{side}_lamp_white",
            (0.004, 0.032, 0.024),
            (post_x + 0.047, y, lamp_z - 0.018),
            WHITE,
            collision=False,
        )
        for axle, x in [("front", d["front_axle_x_m"]), ("rear", d["rear_axle_x_m"])]:
            radius = d["wheel_radius_m"]
            # A faceted arch avoids filling the wheel cutout with a rectangular body.
            for section in range(10):
                angle = (section + 0.5) * math.pi / 10
                arch_radius = radius + 0.023
                box(
                    base,
                    f"{axle}_{side}_fender_{section}",
                    (arch_radius * math.pi / 10 + 0.004, 0.10, 0.018),
                    (
                        x + arch_radius * math.cos(angle),
                        sign * (width / 2 - 0.05),
                        radius + arch_radius * math.sin(angle),
                    ),
                    YELLOW,
                    (0, math.pi / 2 - angle, 0),
                    False,
                )
    # Slotted yellow roof. Its top defines catalogue height exactly.
    for index, x in enumerate([-0.39, -0.285, -0.18, -0.075, 0.03, 0.135]):
        box(
            base,
            f"roof_slat_{index}",
            (0.062, width - 0.103, 0.024),
            (x, 0, height - 0.012),
        )
    for sign in [-1, 1]:
        box(
            base,
            f"roof_edge_{sign}",
            (0.59, 0.033, 0.025),
            (-0.125, sign * (width / 2 - 0.035), height - 0.0125),
        )

    # Steering wheel is a ring assembled from straight segments in a tilted plane.
    tilt = 0.42
    for index in range(24):
        u, v = index * math.tau / 24, (index + 1) * math.tau / 24
        points = [
            (
                0.205 + 0.092 * math.cos(t) * math.cos(tilt),
                0.092 * math.sin(t),
                0.545 + 0.092 * math.cos(t) * math.sin(tilt),
            )
            for t in [u, v]
        ]
        beam(base, f"steering_rim_{index}", *points, 0.018, collision=False)
    beam(
        base,
        "steering_column",
        (0.27, 0, 0.43),
        (0.205, 0, 0.545),
        0.035,
        collision=False,
    )
    for angle in [0, 2 * math.pi / 3, 4 * math.pi / 3]:
        beam(
            base,
            f"steering_spoke_{angle}",
            (0.205, 0, 0.545),
            (
                0.205 + 0.085 * math.cos(angle) * math.cos(tilt),
                0.085 * math.sin(angle),
                0.545 + 0.085 * math.cos(angle) * math.sin(tilt),
            ),
            0.016,
            collision=False,
        )
    for sign in [-1, 1]:
        beam(
            base,
            f"control_lever_{sign}",
            (0.27, sign * 0.18, 0.46),
            (0.22, sign * 0.18, 0.55),
            0.015,
            collision=False,
        )

    # Stationary outer mast, with a moving carriage in front.
    mast_x, mast_height = d["mast_x_m"], d["mast_height_m"]
    for sign in [-1, 1]:
        box(
            base,
            f"mast_rail_{sign}",
            (0.052, 0.04, mast_height - 0.05),
            (mast_x, sign * (d["mast_width_m"] / 2 - 0.02), (mast_height + 0.05) / 2),
            BLACK,
        )
    for index, z in enumerate([0.085, 0.40, mast_height - 0.02]):
        box(
            base,
            f"mast_cross_{index}",
            (0.05, d["mast_width_m"], 0.04),
            (mast_x, 0, z),
            BLACK,
        )
    box(base, "lift_drive_cover", (0.058, 0.09, 0.32), (mast_x, 0, 0.26), GREY)
    for index in range(9):
        box(
            base,
            f"mast_scale_{index}",
            (0.002, 0.035, 0.008),
            (mast_x + 0.027, -0.15, 0.47 + index * 0.018),
            YELLOW,
            collision=False,
        )

    for axle, x in [("front", d["front_axle_x_m"]), ("rear", d["rear_axle_x_m"])]:
        for side, sign in [("left", 1), ("right", -1)]:
            position = (x, sign * d["wheel_track_m"] / 2, d["wheel_radius_m"])
            parent = "base_link"
            if axle == a["steering_axle"]:
                carrier = Link(
                    f"{side}_steering_carrier",
                    parent,
                    position,
                    a["steering_carrier_mass_kg"],
                    (0.025, 0.025, 0.025),
                    joint_name=f"{side}_steer",
                    joint_type="revolute",
                    limits=(-a["steering_limit_rad"], a["steering_limit_rad"]),
                )
                links.append(carrier)
                parent, position = carrier.name, (0, 0, 0)
            wheel = Link(
                f"{axle}_{side}_wheel",
                parent,
                position,
                a["wheel_mass_kg"],
                (
                    d["wheel_radius_m"] * 1.73,
                    d["wheel_width_m"],
                    d["wheel_radius_m"] * 1.73,
                ),
                joint_name=f"{axle}_{side}_spin",
                joint_type="continuous",
                axis=(0, 1, 0),
            )
            links.append(wheel)
            radius, depth = d["wheel_radius_m"], d["wheel_width_m"]
            cylinder(
                wheel,
                f"{axle}_{side}_tire",
                radius,
                depth,
                (0, 0, 0),
                RUBBER,
                (math.pi / 2, 0, 0),
                True,
            )
            for suffix, r, protrusion, color in [
                ("ring", radius * 0.76, 0.002, YELLOW),
                ("hub", radius * 0.66, 0.004, BLACK),
                ("cap", radius * 0.23, 0.006, GREY),
            ]:
                cylinder(
                    wheel,
                    f"{axle}_{side}_{suffix}",
                    r,
                    0.004,
                    (0, sign * (depth / 2 + protrusion), 0),
                    color,
                    (math.pi / 2, 0, 0),
                )
            for index in range(8):
                angle = index * math.tau / 8
                cylinder(
                    wheel,
                    f"{axle}_{side}_bolt_{index}",
                    0.007,
                    0.004,
                    (
                        0.075 * math.cos(angle),
                        sign * (depth / 2 + 0.007),
                        0.075 * math.sin(angle),
                    ),
                    GREY,
                    (math.pi / 2, 0, 0),
                )

    fork = Link(
        "fork_carriage",
        "base_link",
        (0, 0, 0),
        a["carriage_mass_kg"],
        (0.42, d["carriage_width_m"], d["carriage_height_m"]),
        (0.59, 0, 0.15),
        "fork_lift",
        "prismatic",
        (0, 0, 1),
        (0, d["lift_travel_m"]),
    )
    links.append(fork)
    root_x, tip_x = d["fork_root_x_m"], rear + c["overall_length_m"]
    fork_z, back_h = d["fork_center_height_m"], d["carriage_height_m"]
    for side, sign in [("left", 1), ("right", -1)]:
        y = sign * d["fork_spacing_m"] / 2
        box(
            fork,
            f"{side}_fork",
            (tip_x - root_x, d["fork_width_m"], d["fork_thickness_m"]),
            ((tip_x + root_x) / 2, y, fork_z),
            BLACK,
        )
        box(
            fork,
            f"{side}_fork_heel",
            (0.026, d["fork_width_m"], back_h * 0.55),
            (root_x, y, fork_z + back_h * 0.275),
            BLACK,
        )
        fork.sites[f"{side}_fork_tip"] = (tip_x, y, fork_z)
        box(
            fork,
            f"carriage_side_{side}",
            (0.026, 0.035, back_h),
            (root_x, sign * (d["carriage_width_m"] / 2 - 0.0175), fork_z + back_h / 2),
            BLACK,
        )
    for index, z in enumerate([fork_z + 0.045, fork_z + 0.26, fork_z + back_h - 0.02]):
        box(
            fork,
            f"carriage_cross_{index}",
            (0.028, d["carriage_width_m"], 0.032),
            (root_x, 0, z),
            BLACK,
        )
    for index, y in enumerate([-0.16, -0.08, 0, 0.08, 0.16]):
        box(
            fork,
            f"carriage_grille_{index}",
            (0.02, 0.012, 0.13),
            (root_x, y, fork_z + back_h - 0.08),
            BLACK,
        )
    return links
