#!/usr/bin/env python3
"""Minimal negative/replay checks for the Phase-19 planar model builder."""

from __future__ import annotations

import importlib.util
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BUILDER = ROOT / "tools/experiments/build_mujoco_planar_model.py"
SOURCE = ROOT / "simulation/mujoco/model/wheel_leg.xml"
SCENE = ROOT / "simulation/mujoco/model/phase18_floating_contact.xml"


def load_builder():
    spec = importlib.util.spec_from_file_location("phase19_planar_builder", BUILDER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    builder = load_builder()
    with tempfile.TemporaryDirectory(prefix="phase19-planar-test-") as directory:
        output = Path(directory)
        first, second = output / "first.xml", output / "second.xml"
        builder.derive(SOURCE, first)
        builder.derive(SOURCE, second)
        assert first.read_bytes() == second.read_bytes()
        assert builder.comparable_xml(SOURCE, derived=False) == builder.comparable_xml(
            first, derived=True
        )

        malformed = output / "malformed.xml"
        tree = ET.parse(SOURCE)
        base = tree.find("./worldbody/body[@name='base_body']")
        assert base is not None
        base.insert(0, ET.Element("freejoint"))
        tree.write(malformed, encoding="utf-8", xml_declaration=True)
        try:
            builder.derive(malformed, output / "must_not_exist.xml")
        except RuntimeError as error:
            assert "one direct" in str(error)
        else:
            raise AssertionError("Duplicate base freejoint was accepted")

        bad_scene = output / "bad_scene.xml"
        tree = ET.parse(SCENE)
        include = tree.find("./include")
        assert include is not None
        include.set("file", "unexpected.xml")
        tree.write(bad_scene, encoding="utf-8", xml_declaration=True)
        try:
            builder.derive_scene(bad_scene, output / "must_not_exist_scene.xml")
        except RuntimeError as error:
            assert "wheel_leg.xml" in str(error)
        else:
            raise AssertionError("Unexpected scene include was accepted")

    print("Phase 19 planar model builder self-test: PASS")


if __name__ == "__main__":
    main()
