from __future__ import annotations

import pathlib
import tempfile
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def _as_mujoco_path(p: pathlib.Path) -> str:
    """
    MuJoCo accepts forward slashes on all platforms; using as_posix()
    avoids issues with backslashes inside XML attributes.
    """

    return p.as_posix()


def patch_xml_mesh_paths(xml_path: pathlib.Path) -> pathlib.Path:
    """
    Create a patched copy of a MuJoCo XML that reliably resolves mesh files.

    Why:
      - In ROS 2 with --symlink-install, package "share" paths may point into src/.
      - The source tree layout does NOT contain "share/<pkg>/".
      - MuJoCo resolves <mesh file="..."> relative to the XML (or compiler meshdir).

    What we do:
      - Discover openarm_description share dir via ament_index.
      - Set/override <compiler meshdir="..."> to openarm_description/meshes.
      - Rewrite every <mesh file="..."> to be relative to that meshdir.
    """

    xml_path = pathlib.Path(xml_path).resolve()
    if not xml_path.exists():
        raise FileNotFoundError(f"MuJoCo xml not found: {xml_path}")

    openarm_desc_share = pathlib.Path(get_package_share_directory("openarm_description"))
    mesh_base = (openarm_desc_share / "meshes").resolve()
    if not mesh_base.exists():
        raise FileNotFoundError(f"Mesh directory not found: {mesh_base}")

    xml_bytes = xml_path.read_bytes()
    root = ET.fromstring(xml_bytes)

    # Ensure <compiler> exists and force meshdir to our discovered mesh_base
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", _as_mujoco_path(mesh_base))

    # Rewrite mesh file attributes.
    # Typical current values are:
    #   ../../openarm_description/share/openarm_description/meshes/body/v10/visual/body_link0.stl
    # We convert them into:
    #   body/v10/visual/body_link0.stl
    for mesh_el in root.findall(".//mesh"):
        file_attr = mesh_el.get("file")
        if not file_attr:
            continue

        # If it already uses a meshes/ suffix, strip everything up to meshes/
        marker = "meshes/"
        if marker in file_attr:
            rel = file_attr.split(marker, 1)[1].lstrip("/\\")
            mesh_el.set("file", rel)
            continue

        # Otherwise try to resolve relative to XML and then relativize to mesh_base
        candidate = (xml_path.parent / file_attr).resolve()
        try:
            rel_path = candidate.relative_to(mesh_base)
        except Exception:
            # Leave untouched if we can't safely rewrite.
            continue
        mesh_el.set("file", _as_mujoco_path(rel_path))

    patched_dir = pathlib.Path(tempfile.gettempdir()) / "openarm_mujoco"
    patched_dir.mkdir(parents=True, exist_ok=True)
    patched_path = patched_dir / f"{xml_path.stem}.patched.{int(time.time()*1000)}.xml"

    ET.ElementTree(root).write(patched_path, encoding="utf-8", xml_declaration=False)
    return patched_path


