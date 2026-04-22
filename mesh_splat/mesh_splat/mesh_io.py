from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from pcl_msgs.msg import PolygonMesh
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2


@dataclass
class DecodedMesh:
    vertex_positions: np.ndarray
    triangle_indices: np.ndarray
    vertex_texcoords: Optional[np.ndarray]
    vertex_normals: Optional[np.ndarray]
    stamp_ns: int


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def decode_mesh(msg: PolygonMesh) -> DecodedMesh:
    field_names = {field.name for field in msg.cloud.fields}
    required_fields = {"x", "y", "z"}
    missing_fields = sorted(required_fields - field_names)
    if missing_fields:
        raise ValueError(
            f"Mesh point cloud is missing required fields: {missing_fields}"
        )

    available_fields = ["x", "y", "z"]
    has_normals = {"normal_x", "normal_y", "normal_z"}.issubset(field_names)
    has_texcoords = {"u", "v"}.issubset(field_names)
    if has_normals:
        available_fields.extend(["normal_x", "normal_y", "normal_z"])
    if has_texcoords:
        available_fields.extend(["u", "v"])

    points = point_cloud2.read_points(
        msg.cloud, field_names=available_fields, skip_nans=False
    )
    if points.size == 0:
        raise ValueError("Mesh point cloud is empty.")

    vertex_positions = np.stack(
        [points["x"], points["y"], points["z"]], axis=-1
    ).astype(np.float32, copy=False)

    vertex_normals = None
    if has_normals:
        vertex_normals = np.stack(
            [points["normal_x"], points["normal_y"], points["normal_z"]],
            axis=-1,
        ).astype(np.float32, copy=False)

    vertex_texcoords = None
    if has_texcoords:
        vertex_texcoords = np.stack([points["u"], points["v"]], axis=-1).astype(
            np.float32, copy=False
        )

    triangle_indices = np.asarray(
        [polygon.vertices for polygon in msg.polygons if len(polygon.vertices) == 3],
        dtype=np.uint32,
    )
    if triangle_indices.size == 0:
        raise ValueError("Mesh polygon list does not contain any triangles.")

    triangle_indices = triangle_indices.reshape((-1, 3))
    valid_mask = np.all(triangle_indices < vertex_positions.shape[0], axis=1)
    triangle_indices = triangle_indices[valid_mask]
    if triangle_indices.size == 0:
        raise ValueError("Mesh triangles reference invalid vertex indices.")

    return DecodedMesh(
        vertex_positions=vertex_positions,
        triangle_indices=triangle_indices,
        vertex_texcoords=vertex_texcoords,
        vertex_normals=vertex_normals,
        stamp_ns=stamp_to_ns(msg.header.stamp),
    )


def decode_image(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding in {"mono8", "8uc1"}:
        rows = data.reshape((msg.height, msg.step))
        return rows[:, : msg.width].copy()

    if encoding in {"rgb8", "8uc3"}:
        rows = data.reshape((msg.height, msg.step))
        return rows[:, : msg.width * 3].reshape((msg.height, msg.width, 3)).copy()

    if encoding == "bgr8":
        rows = data.reshape((msg.height, msg.step))
        image = rows[:, : msg.width * 3].reshape((msg.height, msg.width, 3))
        return image[:, :, ::-1].copy()

    if encoding in {"rgba8", "8uc4"}:
        rows = data.reshape((msg.height, msg.step))
        return rows[:, : msg.width * 4].reshape((msg.height, msg.width, 4)).copy()

    if encoding == "bgra8":
        rows = data.reshape((msg.height, msg.step))
        image = rows[:, : msg.width * 4].reshape((msg.height, msg.width, 4))
        return image[:, :, [2, 1, 0, 3]].copy()

    raise ValueError(
        f"Unsupported image encoding '{msg.encoding}', expected mono8/rgb8/bgr8/rgba8/bgra8."
    )


def decode_rerun_texture(msg: Image) -> np.ndarray:
    image = decode_image(msg)
    if image.ndim == 2:
        return np.repeat(image[:, :, None], 3, axis=2)
    return image


def image_suffix_for_array(image: np.ndarray) -> str:
    if image.ndim == 2:
        return ".pgm"
    if image.ndim != 3:
        raise ValueError(f"Unsupported image rank {image.ndim}; expected 2 or 3.")
    if image.shape[2] == 1:
        return ".pgm"
    if image.shape[2] == 3:
        return ".ppm"
    if image.shape[2] == 4:
        return ".pam"
    raise ValueError(
        f"Unsupported image channel count {image.shape[2]}; expected 1, 3, or 4."
    )


def write_image_file(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    image = np.asarray(image, dtype=np.uint8)

    if image.ndim == 3 and image.shape[2] == 1:
        image = image[:, :, 0]

    if image.ndim == 2:
        header = f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
        payload = image.tobytes()
    elif image.ndim == 3 and image.shape[2] == 3:
        header = f"P6\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
        payload = image.tobytes()
    elif image.ndim == 3 and image.shape[2] == 4:
        header = (
            "P7\n"
            f"WIDTH {image.shape[1]}\n"
            f"HEIGHT {image.shape[0]}\n"
            "DEPTH 4\n"
            "MAXVAL 255\n"
            "TUPLTYPE RGB_ALPHA\n"
            "ENDHDR\n"
        ).encode("ascii")
        payload = image.tobytes()
    else:
        raise ValueError(
            f"Unsupported image shape {image.shape}; expected (H, W), (H, W, 3), or (H, W, 4)."
        )

    with path.open("wb") as handle:
        handle.write(header)
        handle.write(payload)


def write_obj_mesh(
    obj_path: Path, mesh: DecodedMesh, texture_filename: Optional[str] = None
) -> Optional[Path]:
    obj_path.parent.mkdir(parents=True, exist_ok=True)

    has_texcoords = (
        mesh.vertex_texcoords is not None
        and mesh.vertex_texcoords.shape[0] == mesh.vertex_positions.shape[0]
    )
    has_normals = (
        mesh.vertex_normals is not None
        and mesh.vertex_normals.shape[0] == mesh.vertex_positions.shape[0]
    )

    mtl_path: Optional[Path] = None
    lines: list[str] = []
    if texture_filename is not None:
        mtl_path = obj_path.with_suffix(".mtl")
        lines.append(f"mtllib {mtl_path.name}")
        lines.append("usemtl material0")

    for vertex in mesh.vertex_positions:
        lines.append(f"v {vertex[0]:.8f} {vertex[1]:.8f} {vertex[2]:.8f}")

    if has_texcoords:
        for texcoord in mesh.vertex_texcoords:
            lines.append(f"vt {texcoord[0]:.8f} {texcoord[1]:.8f}")

    if has_normals:
        for normal in mesh.vertex_normals:
            lines.append(f"vn {normal[0]:.8f} {normal[1]:.8f} {normal[2]:.8f}")

    for triangle in mesh.triangle_indices:
        face_tokens = []
        for index in triangle + 1:
            if has_texcoords and has_normals:
                face_tokens.append(f"{index}/{index}/{index}")
            elif has_texcoords:
                face_tokens.append(f"{index}/{index}")
            elif has_normals:
                face_tokens.append(f"{index}//{index}")
            else:
                face_tokens.append(str(index))
        lines.append("f " + " ".join(face_tokens))

    obj_path.write_text("\n".join(lines) + "\n", encoding="ascii")

    if mtl_path is not None:
        mtl_path.write_text(
            "\n".join(
                [
                    "newmtl material0",
                    "Ka 1.000000 1.000000 1.000000",
                    "Kd 1.000000 1.000000 1.000000",
                    "Ks 0.000000 0.000000 0.000000",
                    "d 1.0",
                    "illum 1",
                    f"map_Kd {texture_filename}",
                ]
            )
            + "\n",
            encoding="ascii",
        )

    return mtl_path
