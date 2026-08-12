#!/usr/bin/env python3
"""Convert the Unitree G1 URDF to a USD carrying its LiDAR and camera mounts.

The URDF already declares ``mid360_link`` and ``d435_link`` as fixed children of
``torso_link``, so no mounts need authoring - they only need to *survive*
conversion. That is why fixed-joint merging is disabled: with it on, the
importer collapses those links into the torso and the sensor prim paths vanish.

    python scripts/convert_g1_urdf_to_usd.py
"""

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

REPO = Path(__file__).resolve().parent.parent
DEFAULT_URDF = (
    REPO
    / "assets/robot/g1_29/g1_29dof.urdf"
)
DEFAULT_USD = REPO / "assets/g1_29dof_sensors.usd"

parser = argparse.ArgumentParser(description="Convert G1 URDF to USD with sensor mounts.")
parser.add_argument("--urdf", type=str, default=str(DEFAULT_URDF))
parser.add_argument("--output", type=str, default=str(DEFAULT_USD))
parser.add_argument("--force", action="store_true", help="Reconvert even if the USD exists.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import sys

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

# Links that must exist in the output for the sensors to have somewhere to attach.
REQUIRED_LINKS = ("torso_link", "mid360_link", "d435_link", "pelvis")


def main() -> None:
    urdf_path = Path(args_cli.urdf)
    usd_path = Path(args_cli.output)

    if not urdf_path.exists():
        print(f"[CONVERT] URDF not found: {urdf_path}")
        raise SystemExit(1)

    usd_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[CONVERT] {urdf_path}")
    print(f"[CONVERT] -> {usd_path}")

    cfg = UrdfConverterCfg(
        asset_path=str(urdf_path),
        usd_dir=str(usd_path.parent),
        usd_file_name=usd_path.name,
        fix_base=False,  # free-floating humanoid
        # Critical: keep fixed joints so mid360_link / d435_link remain prims.
        merge_fixed_joints=False,
        self_collision=False,
        # Meshes are detailed; convex hulls keep the collision cost sane.
        collider_type="convex_hull",
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            drive_type="force",
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=100.0, damping=10.0),
        ),
        force_usd_conversion=args_cli.force,
    )

    converter = UrdfConverter(cfg)
    print(f"[CONVERT] written: {converter.usd_path}")

    # Verify the sensor mounts survived rather than trusting the importer.
    from pxr import Usd

    stage = Usd.Stage.Open(converter.usd_path)
    links = {prim.GetName() for prim in stage.Traverse()}

    missing = [name for name in REQUIRED_LINKS if name not in links]
    for name in REQUIRED_LINKS:
        print(f"[CONVERT] {'OK  ' if name in links else 'MISS'} {name}")

    if missing:
        print(f"\n[CONVERT] FAIL: missing links {missing}")
        print("[CONVERT] merge_fixed_joints likely collapsed them into the parent body.")
        raise SystemExit(1)

    joints = [p for p in stage.Traverse() if "Joint" in p.GetTypeName() and "Fixed" not in p.GetTypeName()]
    print(f"[CONVERT] articulated joints: {len(joints)} (expect 29)")
    print("\n[CONVERT] PASS")


if __name__ == "__main__":
    main()
    simulation_app.close()
