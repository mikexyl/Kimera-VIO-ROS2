from __future__ import annotations

from contextlib import suppress
import time
from typing import Optional

import numpy as np
import rerun as rr
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from pcl_msgs.msg import PolygonMesh
from sensor_msgs.msg import Image

from mesh_splat.mesh_io import DecodedMesh, decode_mesh, decode_rerun_texture, stamp_to_ns


class MeshSplatNode(Node):
    def __init__(self) -> None:
        super().__init__("mesh_splat")

        self.declare_parameter("mesh_topic", "mesh")
        self.declare_parameter("texture_topic", "debug_mesh_img")
        self.declare_parameter("entity_path", "map/mesh_splat/mesh")
        self.declare_parameter("rerun_url", "rerun+http://127.0.0.1:9876/proxy")
        self.declare_parameter("application_id", "mesh_splat")
        self.declare_parameter("wireframe_radius", 0.002)

        self.mesh_topic = self.get_parameter("mesh_topic").get_parameter_value().string_value
        self.texture_topic = (
            self.get_parameter("texture_topic").get_parameter_value().string_value
        )
        self.entity_path = self.get_parameter("entity_path").get_parameter_value().string_value
        self.rerun_url = self.get_parameter("rerun_url").get_parameter_value().string_value
        self.application_id = (
            self.get_parameter("application_id").get_parameter_value().string_value
        )
        self.wireframe_radius = (
            self.get_parameter("wireframe_radius").get_parameter_value().double_value
        )

        rr.init(self.application_id, spawn=False)
        rr.connect_grpc(self.rerun_url)
        rr.log("map", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PolygonMesh, self.mesh_topic, self._mesh_callback, qos)
        self.create_subscription(Image, self.texture_topic, self._image_callback, qos)

        self._latest_mesh: Optional[DecodedMesh] = None
        self._latest_texture: Optional[np.ndarray] = None
        self._latest_texture_stamp_ns: Optional[int] = None
        self._last_logged_key: Optional[tuple[int, Optional[int]]] = None
        self._missing_mesh_fields_warned = False
        self._unsupported_image_warned = False
        self._logged_textured_once = False
        self._logged_geometry_once = False

        self.get_logger().info(
            f"mesh_splat listening on mesh='{self.mesh_topic}' "
            f"texture='{self.texture_topic}' and logging to '{self.entity_path}'."
        )

    def _mesh_callback(self, msg: PolygonMesh) -> None:
        try:
            decoded_mesh = decode_mesh(msg)
        except ValueError as exc:
            if not self._missing_mesh_fields_warned:
                self.get_logger().warning(str(exc))
                self._missing_mesh_fields_warned = True
            return
        if decoded_mesh is None:
            return
        self._latest_mesh = decoded_mesh
        self._maybe_log_mesh()

    def _image_callback(self, msg: Image) -> None:
        try:
            texture = decode_rerun_texture(msg)
        except ValueError as exc:
            if not self._unsupported_image_warned:
                self.get_logger().warning(str(exc))
                self._unsupported_image_warned = True
            return
        if texture is None:
            return
        self._latest_texture = texture
        self._latest_texture_stamp_ns = stamp_to_ns(msg.header.stamp)
        self._maybe_log_mesh()

    def _maybe_log_mesh(self) -> None:
        if self._latest_mesh is None:
            return

        texture_stamp_ns = self._latest_texture_stamp_ns if self._latest_texture is not None else None
        log_key = (self._latest_mesh.stamp_ns, texture_stamp_ns)
        if log_key == self._last_logged_key:
            return

        mesh = self._latest_mesh
        rr.set_time("time", timestamp=np.datetime64(time.time_ns(), "ns"))
        mesh_args = {
            "vertex_positions": mesh.vertex_positions,
            "triangle_indices": mesh.triangle_indices,
        }
        if mesh.vertex_normals is not None:
            mesh_args["vertex_normals"] = mesh.vertex_normals
        if mesh.vertex_texcoords is not None and self._latest_texture is not None:
            mesh_args["vertex_texcoords"] = mesh.vertex_texcoords
            mesh_args["albedo_texture"] = self._latest_texture
            if not self._logged_textured_once:
                self._logged_textured_once = True
                self.get_logger().info(
                    "Logging textured mesh with "
                    f"{mesh.vertex_positions.shape[0]} vertices and "
                    f"{mesh.triangle_indices.shape[0]} triangles."
                )
        elif not self._logged_geometry_once:
            self._logged_geometry_once = True
            self.get_logger().info(
                "Logging geometry-only mesh with "
                f"{mesh.vertex_positions.shape[0]} vertices and "
                f"{mesh.triangle_indices.shape[0]} triangles."
            )

        rr.log(self.entity_path, rr.Mesh3D(**mesh_args))

        wireframe = self._build_wireframe(mesh.vertex_positions, mesh.triangle_indices)
        if wireframe is not None:
            colors = np.tile(
                np.array([[0, 0, 0, 220]], dtype=np.uint8), (wireframe.shape[0], 1)
            )
            radii = np.full((wireframe.shape[0],), self.wireframe_radius, dtype=np.float32)
            rr.log(
                f"{self.entity_path}/wireframe",
                rr.LineStrips3D(wireframe, colors=colors, radii=radii),
            )

        self._last_logged_key = log_key

    @staticmethod
    def _build_wireframe(
        vertex_positions: np.ndarray, triangle_indices: np.ndarray
    ) -> Optional[np.ndarray]:
        wireframe_segments = []
        rounded_positions = np.round(vertex_positions, 6)
        seen_edges: set[tuple[tuple[float, float, float], tuple[float, float, float]]] = set()

        for triangle in triangle_indices:
            edges = ((triangle[0], triangle[1]), (triangle[1], triangle[2]), (triangle[2], triangle[0]))
            for first, second in edges:
                if first >= vertex_positions.shape[0] or second >= vertex_positions.shape[0]:
                    continue
                first_key = tuple(float(value) for value in rounded_positions[first])
                second_key = tuple(float(value) for value in rounded_positions[second])
                edge_key = tuple(sorted((first_key, second_key)))
                if edge_key in seen_edges:
                    continue
                seen_edges.add(edge_key)
                wireframe_segments.append(vertex_positions[[first, second]])

        if not wireframe_segments:
            return None

        return np.asarray(wireframe_segments, dtype=np.float32)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[MeshSplatNode] = None
    try:
        node = MeshSplatNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        if node is not None:
            with suppress(Exception, KeyboardInterrupt):
                node.destroy_node()
        with suppress(Exception, KeyboardInterrupt):
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
