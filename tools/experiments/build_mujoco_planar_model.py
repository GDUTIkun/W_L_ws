#!/usr/bin/env python3
"""Derive and audit the Phase-19 exact sagittal MuJoCo plant."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = ROOT / "simulation/mujoco/model/wheel_leg.xml"
DEFAULT_SOURCE_SCENE = ROOT / "simulation/mujoco/model/phase18_floating_contact.xml"
DEFAULT_OUTPUT = (
    ROOT
    / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-model"
)
PLANAR_JOINTS = (
    {"name": "base_x_joint", "type": "slide", "axis": "1 0 0"},
    {"name": "base_z_joint", "type": "slide", "axis": "0 0 1"},
    {"name": "base_pitch_joint", "type": "hinge", "axis": "0 1 0"},
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def required_single(root: ET.Element, path: str, description: str) -> ET.Element:
    matches = root.findall(path)
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one {description}, found {len(matches)}"
        )
    return matches[0]


def derive(source: Path, destination: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    base = required_single(root, "./worldbody/body[@name='base_body']", "base_body")
    freejoints = [child for child in list(base) if child.tag == "freejoint"]
    if len(freejoints) != 1 or freejoints[0].attrib:
        raise RuntimeError("base_body must contain one direct, attribute-free freejoint")
    index = list(base).index(freejoints[0])
    base.remove(freejoints[0])
    for offset, attributes in enumerate(PLANAR_JOINTS):
        base.insert(index + offset, ET.Element("joint", attributes))
    compiler = required_single(root, "./compiler", "compiler")
    compiler.set("meshdir", str((source.parent / "assets").resolve()))
    root.set("model", "wheel_leg_phase19_exact_planar")
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def derive_scene(source_scene: Path, destination: Path) -> None:
    tree = ET.parse(source_scene)
    root = tree.getroot()
    include = required_single(root, "./include", "model include")
    if Path(include.attrib.get("file", "")).name != "wheel_leg.xml":
        raise RuntimeError("Source scene must include wheel_leg.xml")
    include.set("file", "wheel_leg_planar.xml")
    root.set("model", "wheel_leg_phase19_exact_planar_scene")
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def comparable_xml(source: Path, *, derived: bool) -> tuple[Any, ...]:
    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib.pop("model", None)
    compiler = required_single(root, "./compiler", "compiler")
    compiler.attrib.pop("meshdir", None)
    if not derived:
        base = required_single(root, "./worldbody/body[@name='base_body']", "base_body")
        freejoint = [child for child in list(base) if child.tag == "freejoint"]
        if len(freejoint) != 1:
            raise RuntimeError("Source comparison requires one base freejoint")
        index = list(base).index(freejoint[0])
        base.remove(freejoint[0])
        for offset, attributes in enumerate(PLANAR_JOINTS):
            base.insert(index + offset, ET.Element("joint", attributes))

    def canonical(element: ET.Element) -> tuple[Any, ...]:
        return (
            element.tag,
            tuple(sorted(element.attrib.items())),
            (element.text or "").strip(),
            tuple(canonical(child) for child in list(element)),
        )

    return canonical(root)


def names(model: mujoco.MjModel, kind: mujoco.mjtObj, count: int) -> list[str]:
    return [mujoco.mj_id2name(model, kind, index) or "" for index in range(count)]


def maximum_difference(first: np.ndarray, second: np.ndarray) -> float:
    if first.shape != second.shape:
        return math.inf
    return float(np.max(np.abs(first - second))) if first.size else 0.0


def named_rows(
    model: mujoco.MjModel, kind: mujoco.mjtObj, count: int, values: np.ndarray
) -> dict[str, np.ndarray]:
    return {
        mujoco.mj_id2name(model, kind, index) or f"__unnamed_{index}":
        np.asarray(values[index]).copy()
        for index in range(count)
    }


def compare_named(
    full: mujoco.MjModel,
    planar: mujoco.MjModel,
    kind: mujoco.mjtObj,
    count_name: str,
    fields: tuple[str, ...],
) -> dict[str, float]:
    full_count = int(getattr(full, count_name))
    planar_count = int(getattr(planar, count_name))
    if full_count != planar_count:
        raise RuntimeError(f"{count_name} changed: {full_count} != {planar_count}")
    full_names = names(full, kind, full_count)
    planar_names = names(planar, kind, planar_count)
    if full_names != planar_names:
        raise RuntimeError(f"{count_name} names/order changed")
    differences: dict[str, float] = {}
    for field in fields:
        differences[field] = maximum_difference(
            np.asarray(getattr(full, field)), np.asarray(getattr(planar, field))
        )
    return differences


def compile_audit(full_scene: Path, planar_scene: Path) -> dict[str, Any]:
    full = mujoco.MjModel.from_xml_path(str(full_scene))
    planar = mujoco.MjModel.from_xml_path(str(planar_scene))
    expected_full_joints = [
        name for name in names(full, mujoco.mjtObj.mjOBJ_JOINT, full.njnt) if name
    ]
    expected_planar_joints = [
        name for name in names(planar, mujoco.mjtObj.mjOBJ_JOINT, planar.njnt) if name
    ]
    non_base_full = expected_full_joints
    non_base_planar = [name for name in expected_planar_joints if name not in {
        joint["name"] for joint in PLANAR_JOINTS
    }]
    if non_base_full != non_base_planar:
        raise RuntimeError("Non-base joint names/order changed")

    differences: dict[str, float] = {}
    comparisons = (
        (mujoco.mjtObj.mjOBJ_BODY, "nbody", (
            "body_mass", "body_inertia", "body_ipos", "body_iquat",
            "body_pos", "body_quat",
        )),
        (mujoco.mjtObj.mjOBJ_GEOM, "ngeom", (
            "geom_type", "geom_contype", "geom_conaffinity", "geom_condim",
            "geom_size", "geom_pos", "geom_quat", "geom_friction",
            "geom_solref", "geom_solimp", "geom_margin", "geom_gap",
        )),
        (mujoco.mjtObj.mjOBJ_SITE, "nsite", (
            "site_type", "site_size", "site_pos", "site_quat",
        )),
        (mujoco.mjtObj.mjOBJ_ACTUATOR, "nu", (
            "actuator_gear", "actuator_gainprm", "actuator_biasprm",
            "actuator_ctrlrange", "actuator_forcerange",
        )),
        (mujoco.mjtObj.mjOBJ_EQUALITY, "neq", (
            "eq_type", "eq_data", "eq_solref", "eq_solimp",
        )),
        (mujoco.mjtObj.mjOBJ_SENSOR, "nsensor", (
            "sensor_type", "sensor_datatype", "sensor_dim", "sensor_cutoff",
            "sensor_noise",
        )),
    )
    for kind, count_name, fields in comparisons:
        differences.update(compare_named(full, planar, kind, count_name, fields))

    # Compare all non-base joint physical fields by name, independent of shifted IDs.
    joint_fields = ("jnt_type", "jnt_axis", "jnt_pos", "jnt_range",
                    "jnt_stiffness", "jnt_margin")
    full_joint_names = names(full, mujoco.mjtObj.mjOBJ_JOINT, full.njnt)
    planar_joint_names = names(planar, mujoco.mjtObj.mjOBJ_JOINT, planar.njnt)
    for field in joint_fields:
        full_rows = named_rows(
            full, mujoco.mjtObj.mjOBJ_JOINT, full.njnt, np.asarray(getattr(full, field))
        )
        planar_rows = named_rows(
            planar, mujoco.mjtObj.mjOBJ_JOINT, planar.njnt,
            np.asarray(getattr(planar, field)),
        )
        differences[field] = max(
            maximum_difference(full_rows[name], planar_rows[name])
            for name in non_base_full
        )

    full_data, planar_data = mujoco.MjData(full), mujoco.MjData(planar)
    mujoco.mj_forward(full, full_data)
    mujoco.mj_forward(planar, planar_data)
    pose_difference = max(
        maximum_difference(full_data.xpos, planar_data.xpos),
        maximum_difference(full_data.xquat, planar_data.xquat),
    )
    option_difference = max(
        abs(float(full.opt.timestep - planar.opt.timestep)),
        maximum_difference(np.asarray(full.opt.gravity), np.asarray(planar.opt.gravity)),
        float(full.opt.integrator != planar.opt.integrator),
        float(full.opt.solver != planar.opt.solver),
        float(full.opt.cone != planar.opt.cone),
        float(full.opt.iterations != planar.opt.iterations),
        float(full.opt.ls_iterations != planar.opt.ls_iterations),
    )
    max_physics_difference = max(differences.values(), default=0.0)
    expected = {
        "full": {"nq": 17, "nv": 16, "njnt": 11},
        "planar": {"nq": 13, "nv": 13, "njnt": 13},
    }
    actual = {
        "full": {"nq": full.nq, "nv": full.nv, "njnt": full.njnt},
        "planar": {"nq": planar.nq, "nv": planar.nv, "njnt": planar.njnt},
    }
    planar_types = {
        name: int(planar.jnt_type[mujoco.mj_name2id(
            planar, mujoco.mjtObj.mjOBJ_JOINT, name
        )])
        for name in (joint["name"] for joint in PLANAR_JOINTS)
    }
    planar_axes = {
        name: planar.jnt_axis[mujoco.mj_name2id(
            planar, mujoco.mjtObj.mjOBJ_JOINT, name
        )].tolist()
        for name in (joint["name"] for joint in PLANAR_JOINTS)
    }
    passed = (
        actual == expected
        and max_physics_difference == 0.0
        and pose_difference == 0.0
        and option_difference == 0.0
        and planar_types == {
            "base_x_joint": int(mujoco.mjtJoint.mjJNT_SLIDE),
            "base_z_joint": int(mujoco.mjtJoint.mjJNT_SLIDE),
            "base_pitch_joint": int(mujoco.mjtJoint.mjJNT_HINGE),
        }
        and planar_axes == {
            "base_x_joint": [1.0, 0.0, 0.0],
            "base_z_joint": [0.0, 0.0, 1.0],
            "base_pitch_joint": [0.0, 1.0, 0.0],
        }
    )
    return {
        "passed": passed,
        "expected_topology": expected,
        "actual_topology": actual,
        "planar_base_joint_types": planar_types,
        "planar_base_joint_axes": planar_axes,
        "max_preserved_compiled_field_difference": max_physics_difference,
        "max_qpos0_body_pose_difference": pose_difference,
        "max_option_difference": option_difference,
        "field_differences": differences,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--source-scene", type=Path, default=DEFAULT_SOURCE_SCENE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    source, source_scene = args.source.resolve(), args.source_scene.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f"Refusing to overwrite non-empty output directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    derived = output_dir / "wheel_leg_planar.xml"
    scene = output_dir / "phase19_planar_scene.xml"
    derive(source, derived)
    derive_scene(source_scene, scene)
    xml_exact = comparable_xml(source, derived=False) == comparable_xml(
        derived, derived=True
    )
    audit = compile_audit(source_scene, scene)
    audit["xml_transform_exact"] = xml_exact
    audit["passed"] = bool(audit["passed"] and xml_exact)
    (output_dir / "audit.json").write_text(
        json.dumps(audit, indent=2) + "\n", encoding="utf-8"
    )
    manifest = {
        "schema_version": 1,
        "phase": 19,
        "profile": "current-nominal-exact-planar",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": sys.version,
        "platform": platform.platform(),
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "inputs": {
            "source": str(source.relative_to(ROOT)),
            "source_scene": str(source_scene.relative_to(ROOT)),
        },
        "outputs": [derived.name, scene.name, "audit.json"],
        "sha256": {
            "source": sha256(source),
            "source_scene": sha256(source_scene),
            "generator": sha256(Path(__file__)),
            "derived": sha256(derived),
            "scene": sha256(scene),
            "audit": sha256(output_dir / "audit.json"),
        },
        "hardware_data": False,
        "supersedes": "phase19-v1-pre-freeze-REWORK",
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(audit, indent=2))
    return 0 if audit["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
