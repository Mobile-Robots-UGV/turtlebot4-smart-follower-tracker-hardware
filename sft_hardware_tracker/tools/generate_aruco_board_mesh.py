#!/usr/bin/env python3
"""Generate Gazebo/RViz visuals for the SFT ArUco board."""

from __future__ import annotations

from pathlib import Path

import cv2


# Simulation visual. The real hardware board config remains in board_pose_ros/config.
BOARD_SIZE_M = 0.30
MARKER_SIZE_M = 0.06
DICTIONARY = cv2.aruco.DICT_6X6_250
MARKERS = {
    1: {"top_left_xy_m": (-0.15, 0.15), "rotation_deg": 0},
    3: {"top_left_xy_m": (0.09, 0.15), "rotation_deg": 0},
    2: {"top_left_xy_m": (0.09, -0.09), "rotation_deg": 180},
    4: {"top_left_xy_m": (-0.15, -0.09), "rotation_deg": 0},
}


def marker_cells(marker_id: int, rotation_deg: int) -> list[list[bool]]:
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    image = cv2.aruco.drawMarker(dictionary, marker_id, 80, borderBits=1)
    cell_px = image.shape[0] // 8
    cells = []

    for row in range(8):
        values = []
        for col in range(8):
            sample = image[row * cell_px + cell_px // 2, col * cell_px + cell_px // 2]
            values.append(int(sample) < 128)
        cells.append(values)

    if rotation_deg == 180:
        cells = [list(reversed(row)) for row in reversed(cells)]
    elif rotation_deg != 0:
        raise ValueError(f"Unsupported marker rotation: {rotation_deg}")

    return cells


def add_quad(vertices: list[tuple[float, float, float]],
             faces: list[tuple[str, tuple[int, int, int, int]]],
             material: str,
             x0: float,
             y0: float,
             x1: float,
             y1: float,
             z: float) -> None:
    start = len(vertices) + 1
    vertices.extend([
        (x0, y0, z),
        (x1, y0, z),
        (x1, y1, z),
        (x0, y1, z),
    ])
    faces.append((material, (start, start + 1, start + 2, start + 3)))


def main() -> None:
    package_dir = Path(__file__).resolve().parents[1]
    mesh_dir = package_dir / "meshes"
    urdf_dir = package_dir / "urdf"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    urdf_dir.mkdir(parents=True, exist_ok=True)

    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[str, tuple[int, int, int, int]]] = []

    half = BOARD_SIZE_M / 2.0
    add_quad(vertices, faces, "white", -half, -half, half, half, 0.0)

    for marker_id, info in MARKERS.items():
        top_left_x, top_left_y = info["top_left_xy_m"]
        cell_size = MARKER_SIZE_M / 8.0
        cells = marker_cells(marker_id, int(info["rotation_deg"]))

        for row, values in enumerate(cells):
            for col, is_black in enumerate(values):
                if not is_black:
                    continue
                x0 = top_left_x + col * cell_size
                x1 = x0 + cell_size
                y1 = top_left_y - row * cell_size
                y0 = y1 - cell_size
                add_quad(vertices, faces, "black", x0, y0, x1, y1, 0.0006)

    obj_path = mesh_dir / "sft_aruco_board.obj"
    mtl_path = mesh_dir / "sft_aruco_board.mtl"
    xacro_path = urdf_dir / "sft_aruco_board_visual.xacro"

    with obj_path.open("w", encoding="utf-8") as obj:
        obj.write("mtllib sft_aruco_board.mtl\n")
        for x, y, z in vertices:
            obj.write(f"v {x:.8f} {y:.8f} {z:.8f}\n")

        current_material = None
        for material, indices in faces:
            if material != current_material:
                obj.write(f"usemtl {material}\n")
                current_material = material
            obj.write("f " + " ".join(str(index) for index in indices) + "\n")

    with mtl_path.open("w", encoding="utf-8") as mtl:
        mtl.write("newmtl white\n")
        mtl.write("Ka 1.0 1.0 1.0\n")
        mtl.write("Kd 1.0 1.0 1.0\n")
        mtl.write("Ks 0.0 0.0 0.0\n")
        mtl.write("newmtl black\n")
        mtl.write("Ka 0.0 0.0 0.0\n")
        mtl.write("Kd 0.0 0.0 0.0\n")
        mtl.write("Ks 0.0 0.0 0.0\n")

    with xacro_path.open("w", encoding="utf-8") as xacro:
        xacro.write('<?xml version="1.0"?>\n')
        xacro.write('<robot xmlns:xacro="http://ros.org/wiki/xacro">\n')
        xacro.write('  <xacro:macro name="sft_aruco_board_visuals">\n')
        xacro.write('    <visual name="sft_aruco_board_white">\n')
        xacro.write('      <origin xyz="0 0 -0.002" rpy="0 0 0"/>\n')
        xacro.write(f'      <geometry><box size="{BOARD_SIZE_M:.8f} {BOARD_SIZE_M:.8f} 0.004"/></geometry>\n')
        xacro.write('      <material name="sft_aruco_white"><color rgba="1 1 1 1"/></material>\n')
        xacro.write('    </visual>\n')

        visual_index = 0
        cell_thickness = 0.001
        cell_z = cell_thickness / 2.0
        for marker_id, info in MARKERS.items():
            top_left_x, top_left_y = info["top_left_xy_m"]
            cell_size = MARKER_SIZE_M / 8.0
            cells = marker_cells(marker_id, int(info["rotation_deg"]))

            for row, values in enumerate(cells):
                for col, is_black in enumerate(values):
                    if not is_black:
                        continue
                    x0 = top_left_x + col * cell_size
                    x1 = x0 + cell_size
                    y1 = top_left_y - row * cell_size
                    y0 = y1 - cell_size
                    cx = (x0 + x1) / 2.0
                    cy = (y0 + y1) / 2.0
                    xacro.write(f'    <visual name="sft_aruco_black_{visual_index:03d}">\n')
                    xacro.write(f'      <origin xyz="{cx:.8f} {cy:.8f} {cell_z:.8f}" rpy="0 0 0"/>\n')
                    xacro.write(f'      <geometry><box size="{cell_size:.8f} {cell_size:.8f} {cell_thickness:.8f}"/></geometry>\n')
                    xacro.write('      <material name="sft_aruco_black"><color rgba="0 0 0 1"/></material>\n')
                    xacro.write('    </visual>\n')
                    visual_index += 1

        xacro.write('  </xacro:macro>\n')
        xacro.write('</robot>\n')

    print(obj_path)
    print(mtl_path)
    print(xacro_path)


if __name__ == "__main__":
    main()
