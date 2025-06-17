from pathlib import Path
import xml.etree.ElementTree as ET
import trimesh                                # pip install trimesh[all]
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401 (activates 3-D backend)
import open3d as o3d

from openai import OpenAI
import json, os

URDF_PATH = Path("meshes/idc_lab_updated.urdf")        # adjust as needed
BASE_DIR  = Path("~/IDC_Lab/src/idc_lab").expanduser()

def load_meshes_from_urdf(
        urdf_file : Path,
        base_dir  : Path,
        *,
        n_points   : int | None = None,   # Poisson-disk target count
        voxel_size : float | None = None  # voxel edge (same units as mesh)
    ):
    """
    Return [{'name': <filename>, 'points': ndarray float32 (K,3)}, …]

    ── Down-sampling rules ───────────────────────────────────────────────
    • If voxel_size is given → voxel-grid down-sample (fast & uniform).
    • Else if n_points is given → Poisson-disk sample that many points.
    • Else → unique vertices (no reduction).

    All coordinates are divided by 1 000  (metres → kilometres etc.).
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    out  = []

    for mesh_tag in root.iter("mesh"):
        fname = mesh_tag.get("filename")
        if not fname:
            continue

        abs_path = (base_dir
                    / Path(fname).expanduser().relative_to(base_dir)).resolve()
        mesh = trimesh.load(abs_path, force="mesh")

        # -------- generate an (N,3) point cloud ------------------------
        if voxel_size is not None:
            # 1) regular voxel grid ↓
            vg   = mesh.voxelized(pitch=voxel_size)
            pts  = vg.points
        elif n_points is not None:
            # 2) Poisson-disk surface sample ↓
            pts, _ = trimesh.sample.sample_surface_even(mesh, n_points)
        else:
            # 3) raw unique vertices
            pts  = mesh.vertices

        # scale & store
        pts = (pts / 1000.0).astype(np.float32)      # km-scale + lean dtype
        out.append({"name": fname, "points": pts})

    return out

# ---------------------------------------------------------------------
# EXAMPLE USAGE
# ---------------------------------------------------------------------
# (b) Voxel grid 2 mm pitch per mesh
mesh_list = load_meshes_from_urdf(URDF_PATH, BASE_DIR, n_points=2000)

# print(f"loaded {len(mesh_list)} entries")
# print(mesh_list[0]["name"], mesh_list[0]["points"].shape)   # (5000, 3)
# print(mesh_list[0]["points"][:40])                           # first 5 XYZ rows


pts = mesh_list[4]["points"]              # (N, 3) NumPy array
# 2) …turn it into a pure-Python structure…
pts_list = pts.tolist()                       # regular nested lists

# 3) …and JSON-encode it to a text string
pts_json = json.dumps(pts_list)                  # NOW it’s serialisable

AI_Agent = OpenAI(api_key=os.getenv("UFL_API_KEY"), base_url="https://api.ai.it.ufl.edu")


response = AI_Agent.chat.completions.create(
    model= "gpt-4o",
    messages=[{
                "role": "user",
                "content": [
                    # --- your prompt & image ---
                    {"type": "text",  "text": "Given the 3d points of an object, describe it"},
                    {"type": "text", "text": pts_json}
                ],
            }],
)

print(response.choices[0].message)

pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
o3d.visualization.draw_geometries([pcd])     # drag to inspect