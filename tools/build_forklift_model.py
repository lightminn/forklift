"""Generate matching URDF/MJCF models from the provisional DLS08 parameters."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

from forklift_model_geometry import (
    Link,
    Part,
    assembly,
    load_parameters,
    validate_assembly_envelope,
)


def numbers(values) -> str:
    return " ".join(f"{number:.10g}" for number in values)


def name_id(name: str) -> str:
    return name.replace("-", "neg_").replace(".", "_")


def quaternion_wxyz(rpy: tuple) -> tuple:
    roll, pitch, yaw = (angle / 2 for angle in rpy)
    cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def diagonal_inertia(link: Link) -> tuple:
    x, y, z = link.inertia_size
    return tuple(
        link.mass_kg / 12 * value
        for value in [y * y + z * z, x * x + z * z, x * x + y * y]
    )


def write_xml(root: ET.Element, path: Path) -> None:
    ET.indent(root, space="  ")
    text = ET.tostring(root, encoding="unicode")
    path.write_text('<?xml version="1.0"?>\n' + text + "\n")


def mjcf_geometry(body: ET.Element, part: Part, collision: bool) -> None:
    size = (
        tuple(value / 2 for value in part.size)
        if part.shape == "box"
        else (part.size[0], part.size[1] / 2)
    )
    ET.SubElement(
        body,
        "geom",
        {
            "name": name_id(part.name) + ("_collision" if collision else "_visual"),
            "type": part.shape,
            "size": numbers(size),
            "pos": numbers(part.pos),
            "quat": numbers(quaternion_wxyz(part.rpy)),
            "rgba": numbers((0.1, 0.65, 0.85, 0.22) if collision else part.color),
            "group": "3" if collision else "1",
            "mass": "0",
            "contype": "1" if collision else "0",
            "conaffinity": "2" if collision else "0",
        },
    )


def export_mjcf(links: list[Link], parameters: dict, output: Path) -> None:
    a = parameters["assumptions"]
    root = ET.Element("mujoco", model=parameters["model_name"])
    ET.SubElement(root, "compiler", angle="radian", inertiafromgeom="false")
    ET.SubElement(
        root,
        "option",
        timestep="0.002",
        integrator="implicitfast",
        gravity="0 0 -9.81",
        cone="elliptic",
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1600", offheight="1200")
    ET.SubElement(visual, "quality", shadowsize="2048", offsamples="4")
    ET.SubElement(visual, "headlight", ambient=".3 .3 .3", diffuse=".65 .65 .65")
    ET.SubElement(root, "statistic", center=".15 0 .48", extent="1.7")
    default = ET.SubElement(root, "default")
    ET.SubElement(
        default,
        "geom",
        friction=f"{a['sliding_friction']} .01 .001",
        solref=".015 1",
        condim="4",
    )
    ET.SubElement(default, "joint", damping=".04", armature=".002")
    custom = ET.SubElement(root, "custom")
    ET.SubElement(
        custom,
        "text",
        name="evidence_status",
        data="candidate_catalogue_dimensions_photo_estimates_unmeasured_dynamics",
    )
    world = ET.SubElement(root, "worldbody")
    bodies = {}
    for link in links:
        parent = world if link.parent is None else bodies[link.parent]
        body = ET.SubElement(parent, "body", name=link.name, pos=numbers(link.pos))
        bodies[link.name] = body
        ET.SubElement(
            body,
            "inertial",
            mass=f"{link.mass_kg:.10g}",
            pos=numbers(link.center_of_mass),
            diaginertia=numbers(diagonal_inertia(link)),
        )
        if link.parent is None:
            ET.SubElement(body, "freejoint", name="floating_base")
        elif link.joint_name:
            attrs = {
                "name": link.joint_name,
                "axis": numbers(link.axis),
                "type": "slide" if link.joint_type == "prismatic" else "hinge",
            }
            if link.limits is not None:
                attrs.update(range=numbers(link.limits), limited="true")
            if link.joint_type == "prismatic":
                attrs.update(damping="25", armature=".02")
            ET.SubElement(body, "joint", attrs)
        for part in link.parts:
            mjcf_geometry(body, part, collision=False)
            if part.collision:
                mjcf_geometry(body, part, collision=True)
        for site, pos in link.sites.items():
            ET.SubElement(
                body,
                "site",
                name=site,
                pos=numbers(pos),
                size=".008",
                rgba="0 .65 .85 1",
                group="4",
            )
    actuators = ET.SubElement(root, "actuator")
    for link in links:
        if link.joint_type == "continuous":
            # Four independent speed actuators are a test interface, not a drive claim.
            limit = a["wheel_speed_limit_radps"]
            torque = a["wheel_torque_limit_nm"]
            ET.SubElement(
                actuators,
                "velocity",
                name=link.joint_name + "_speed",
                joint=link.joint_name,
                kv="1.2",
                ctrllimited="true",
                ctrlrange=numbers((-limit, limit)),
                forcelimited="true",
                forcerange=numbers((-torque, torque)),
            )
        elif link.joint_type == "revolute":
            torque = a["steering_torque_limit_nm"]
            ET.SubElement(
                actuators,
                "position",
                name=link.joint_name + "_position",
                joint=link.joint_name,
                kp="35",
                kv="3",
                ctrllimited="true",
                ctrlrange=numbers(link.limits),
                forcelimited="true",
                forcerange=numbers((-torque, torque)),
            )
        elif link.joint_type == "prismatic":
            force = a["lift_force_limit_n"]
            ET.SubElement(
                actuators,
                "position",
                name="fork_height",
                joint=link.joint_name,
                kp=str(a["lift_position_gain"]),
                kv="60",
                ctrllimited="true",
                ctrlrange=numbers(link.limits),
                forcelimited="true",
                forcerange=numbers((-force, force)),
            )
    write_xml(root, output / "forklift.xml")


def urdf_geometry(node: ET.Element, part: Part) -> None:
    geometry = ET.SubElement(node, "geometry")
    if part.shape == "box":
        ET.SubElement(geometry, "box", size=numbers(part.size))
    else:
        ET.SubElement(
            geometry, "cylinder", radius=str(part.size[0]), length=str(part.size[1])
        )


def export_urdf(links: list[Link], parameters: dict, output: Path) -> None:
    a = parameters["assumptions"]
    root = ET.Element("robot", name=parameters["model_name"])
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(extension, "compiler", discardvisual="false", fusestatic="false")
    for link in links:
        body = ET.SubElement(root, "link", name=link.name)
        inertial = ET.SubElement(body, "inertial")
        ET.SubElement(inertial, "origin", xyz=numbers(link.center_of_mass), rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=str(link.mass_kg))
        ixx, iyy, izz = diagonal_inertia(link)
        ET.SubElement(
            inertial,
            "inertia",
            ixx=str(ixx),
            iyy=str(iyy),
            izz=str(izz),
            ixy="0",
            ixz="0",
            iyz="0",
        )
        for part in link.parts:
            for kind in ["visual", "collision"] if part.collision else ["visual"]:
                node = ET.SubElement(body, kind, name=name_id(part.name) + "_" + kind)
                ET.SubElement(
                    node, "origin", xyz=numbers(part.pos), rpy=numbers(part.rpy)
                )
                urdf_geometry(node, part)
                if kind == "visual":
                    material = ET.SubElement(node, "material", name=name_id(part.name))
                    ET.SubElement(material, "color", rgba=numbers(part.color))
        if link.parent is not None:
            joint = ET.SubElement(
                root, "joint", name=link.joint_name, type=link.joint_type
            )
            ET.SubElement(joint, "parent", link=link.parent)
            ET.SubElement(joint, "child", link=link.name)
            ET.SubElement(joint, "origin", xyz=numbers(link.pos), rpy="0 0 0")
            ET.SubElement(joint, "axis", xyz=numbers(link.axis))
            limits = {}
            if link.limits is not None:
                limits.update(lower=str(link.limits[0]), upper=str(link.limits[1]))
            if link.joint_type == "continuous":
                effort, velocity = (
                    a["wheel_torque_limit_nm"],
                    a["wheel_speed_limit_radps"],
                )
            elif link.joint_type == "revolute":
                effort, velocity = (
                    a["steering_torque_limit_nm"],
                    a["urdf_steering_speed_limit_radps"],
                )
            else:
                effort, velocity = (
                    a["lift_force_limit_n"],
                    a["urdf_lift_speed_limit_mps"],
                )
            limits.update(effort=str(effort), velocity=str(velocity))
            ET.SubElement(joint, "limit", limits)
        for name, pos in link.sites.items():
            ET.SubElement(root, "link", name=name)
            joint = ET.SubElement(root, "joint", name=name + "_fixed", type="fixed")
            ET.SubElement(joint, "parent", link=link.name)
            ET.SubElement(joint, "child", link=name)
            ET.SubElement(joint, "origin", xyz=numbers(pos), rpy="0 0 0")
    write_xml(root, output / "forklift.urdf")


def export_scene(output: Path) -> None:
    root = ET.Element("mujoco", model="dls08_model_check_scene")
    ET.SubElement(root, "include", file="forklift.xml")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        name="sky",
        type="skybox",
        builtin="gradient",
        rgb1=".14 .19 .25",
        rgb2=".38 .43 .49",
        width="512",
        height="3072",
    )
    ET.SubElement(
        asset,
        "texture",
        name="floor_pattern",
        type="2d",
        builtin="checker",
        rgb1=".19 .23 .28",
        rgb2=".22 .26 .31",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset,
        "material",
        name="floor_material",
        texture="floor_pattern",
        texrepeat="6 6",
        reflectance=".08",
    )
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(
        world,
        "light",
        name="key_light",
        pos="1 -2 3",
        dir="-1 2 -3",
        diffuse=".85 .85 .85",
        castshadow="true",
    )
    ET.SubElement(
        world,
        "light",
        name="fill_light",
        pos="-2 1 2",
        dir="2 -1 -2",
        diffuse=".5 .5 .5",
        castshadow="false",
    )
    ET.SubElement(
        world,
        "geom",
        name="ground",
        type="plane",
        size="4 4 .1",
        material="floor_material",
        contype="2",
        conaffinity="1",
    )
    write_xml(root, output / "scene.xml")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parameters", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        parameters = load_parameters(args.parameters)
        links = assembly(parameters)
        validate_assembly_envelope(links, parameters)
    except (ValueError, OSError) as error:
        parser.exit(2, f"{error}\n")
    args.output.mkdir(parents=True, exist_ok=True)
    export_mjcf(links, parameters, args.output)
    export_urdf(links, parameters, args.output)
    export_scene(args.output)
    metadata = {
        "model_name": parameters["model_name"],
        "parameter_sha256": hashlib.sha256(args.parameters.read_bytes()).hexdigest(),
        "evidence": parameters["evidence"],
        "catalogue": parameters["catalogue"],
        "steering_axle_assumption": parameters["assumptions"]["steering_axle"],
        "self_collision": {
            "mjcf": False,
            "urdf": "importer_defined_requires_configuration",
        },
        "urdf_scope": "geometry_and_kinematics_not_equivalent_physics",
        "drive_interface": "four_independent_wheel_speed_actuators_assumed",
        "links": len(links),
        "parts": sum(len(link.parts) for link in links),
        "outputs": ["forklift.xml", "forklift.urdf", "scene.xml"],
    }
    (args.output / "model_manifest.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
