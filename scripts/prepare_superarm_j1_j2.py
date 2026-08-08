#!/usr/bin/env python3
"""Prepare the validated SuperArm geometry with only J1/J2 movable."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from lerobot_robot_roboparty.two_motor_calibration import load_calibration  # noqa: E402

DEFAULT_SOURCE = Path("/home/dong/echo/superarm_ws/robot_arm_hand_package.zip")
DEFAULT_OUTPUT = REPO_ROOT / "third_party/robot_arm_vr/assets/superarm_j1_j2"
ACTIVE_JOINTS = ("joint_rev_1", "joint_rev_2")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def reduce_to_j1_j2(root: ET.Element, calibration: dict) -> None:
    """Keep the full SuperArm link geometry but lock every arm joint after J2."""
    motors = {int(motor["command_id"]): motor for motor in calibration["motors"]}
    if not all(motor_id in motors for motor_id in (1, 2)):
        raise ValueError("calibration must contain motor command IDs 1 and 2")

    root.set("name", "superarm_j1_j2")
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    missing = [name for name in ACTIVE_JOINTS if name not in joints]
    if missing:
        raise ValueError(f"SuperArm source is missing active joints: {missing}")

    for index, name in enumerate(ACTIVE_JOINTS, 1):
        joint = joints[name]
        joint.set("type", "revolute")
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        motor = motors[index]
        limit.set("lower", str(float(motor["range_min_rad"])))
        limit.set("upper", str(float(motor["range_max_rad"])))
        limit.set("effort", "27.0")
        limit.set("velocity", "3.77")

    for joint in root.findall("joint"):
        if joint.get("name") in ACTIVE_JOINTS or joint.get("type") == "fixed":
            continue
        joint.set("type", "fixed")
        for tag in ("axis", "limit", "dynamics", "calibration", "safety_controller", "mimic"):
            child = joint.find(tag)
            if child is not None:
                joint.remove(child)

    # Transmissions and Gazebo plugins are not used by placo/WebXR. Keeping
    # transmissions for joints that are now fixed makes the URDF contradictory.
    for element in list(root):
        if element.tag in {"transmission", "gazebo"}:
            root.remove(element)

    if "hand_mount" not in {link.get("name") for link in root.findall("link")}:
        ET.SubElement(root, "link", {"name": "hand_mount"})
        mount = ET.SubElement(root, "joint", {"name": "hand_mount_fixed", "type": "fixed"})
        ET.SubElement(mount, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(mount, "parent", {"link": "wrist_adapter_hand"})
        ET.SubElement(mount, "child", {"link": "hand_mount"})


def prepare(source_zip: Path, calibration_path: Path, output_dir: Path) -> dict:
    source_zip = source_zip.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    calibration = load_calibration(calibration_path)
    xacro_bin = shutil.which("xacro") or "/opt/ros/humble/bin/xacro"
    if not source_zip.is_file():
        raise FileNotFoundError(source_zip)
    if not Path(xacro_bin).is_file():
        raise FileNotFoundError("xacro executable was not found")

    with tempfile.TemporaryDirectory(prefix="superarm-j1-j2-") as temporary:
        extracted = Path(temporary) / "source"
        with zipfile.ZipFile(source_zip) as archive:
            archive.extractall(extracted)
        candidates = list(extracted.rglob("robot_arm_hand_urdf.xacro"))
        if len(candidates) != 1:
            raise ValueError(f"expected one SuperArm xacro, found {len(candidates)}")
        source_xacro = candidates[0]
        expanded = Path(temporary) / "superarm_full.urdf"
        # ROS Humble's wrapper resolves package metadata through its Python 3.10
        # installation. Running that wrapper from a Python 3.12 venv can inherit a
        # conflicting PYTHONPATH and fail with PackageNotFoundError(xacro). Invoke
        # the installed module with the system interpreter instead.
        subprocess.run(
            ["/usr/bin/python3", xacro_bin, str(source_xacro), "-o", str(expanded)],
            check=True,
        )

        tree = ET.parse(expanded)
        root = tree.getroot()
        reduce_to_j1_j2(root, calibration)

        if output_dir.exists():
            shutil.rmtree(output_dir)
        mesh_output = output_dir / "meshes"
        mesh_output.mkdir(parents=True)
        source_mesh_dir = source_xacro.parent.parent / "meshes"
        copied: set[str] = set()
        for mesh in root.findall(".//mesh"):
            name = Path(mesh.get("filename", "")).name
            source_mesh = source_mesh_dir / name
            if not source_mesh.is_file():
                raise FileNotFoundError(f"missing SuperArm mesh: {source_mesh}")
            if name not in copied:
                shutil.copy2(source_mesh, mesh_output / name)
                copied.add(name)
            mesh.set("filename", f"meshes/{name}")

        ET.indent(tree, space="  ")
        urdf_path = output_dir / "superarm_j1_j2.urdf"
        tree.write(urdf_path, encoding="utf-8", xml_declaration=True)

    movable = [
        joint.get("name")
        for joint in root.findall("joint")
        if joint.get("type") != "fixed"
    ]
    manifest = {
        "schema_version": 1,
        "source_archive": str(source_zip),
        "source_archive_sha256": _sha256(source_zip),
        "calibration": str(calibration_path.resolve()),
        "calibration_captured_at": calibration["captured_at"],
        "urdf": str(urdf_path),
        "urdf_sha256": _sha256(urdf_path),
        "movable_joints": movable,
        "fixed_arm_joints": ["joint_rev_3", "joint_rev_4", "joint_fix_28"],
        "link_count": len(root.findall("link")),
        "mesh_count": len(copied),
        "ee_frame": "hand_mount",
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False, sort_keys=True) + "\n"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument(
        "--calibration", type=Path, default=REPO_ROOT / "config/right_arm_two_motor.json"
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.calibration, args.output), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
