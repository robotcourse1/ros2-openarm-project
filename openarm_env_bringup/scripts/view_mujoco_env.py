#!/usr/bin/env python3
"""
Simple MuJoCo viewer for the OpenArm environment.

Usage:
  python3 view_mujoco_env.py /path/to/openarm_env.xml
If no path provided, it defaults to the installed share path of openarm_env_description.
"""
import argparse
import pathlib
import time

import mujoco
from mujoco import viewer
from ament_index_python.packages import get_package_share_directory

from openarm_env_bringup.mujoco_xml_utils import patch_xml_mesh_paths


def resolve_xml_path(cli_path: str | None) -> pathlib.Path:
    if cli_path:
        return pathlib.Path(cli_path).resolve()
    pkg_share = pathlib.Path(get_package_share_directory("openarm_env_description"))
    return pkg_share / "mujoco" / "openarm_env.xml"


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo viewer for OpenArm env")
    parser.add_argument("xml", nargs="?", help="Path to openarm_env.xml (optional)")
    args = parser.parse_args()

    xml_path = resolve_xml_path(args.xml)
    if not xml_path.exists():
        raise FileNotFoundError(f"MuJoCo xml not found: {xml_path}")

    # Patch XML mesh paths so it works both in install space and in --symlink-install.
    patched_xml_path: pathlib.Path | None = None
    try:
        patched_xml_path = patch_xml_mesh_paths(xml_path)
        print(f"Using patched MuJoCo XML: {patched_xml_path}")
    except Exception as e:
        print(f"Warning: could not patch XML mesh paths: {e}")
        print("Falling back to loading the original XML; mesh loads may fail.")

    # Load model
    # MuJoCo will attempt to load meshes using paths specified in XML
    # If mesh files are not found, MuJoCo may fall back to simplified geometry or show errors
    try:
        model = mujoco.MjModel.from_xml_path(str(patched_xml_path or xml_path))
        data = mujoco.MjData(model)
        print("Model loaded successfully.")
    except Exception as e:
        print(f"Error loading MuJoCo model: {e}")
        print("This might be due to incorrect mesh file paths.")
        raise

    with viewer.launch_passive(model, data) as v:
        last_time = time.time()
        while v.is_running():
            mujoco.mj_step(model, data)
            v.sync()
            # simple rate limit ~500 Hz viewer loop
            dt = time.time() - last_time
            if dt < 0.002:
                time.sleep(0.002 - dt)
            last_time = time.time()


if __name__ == "__main__":
    main()








