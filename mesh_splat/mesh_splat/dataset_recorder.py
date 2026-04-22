from __future__ import annotations

import csv
import json
import re
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from pcl_msgs.msg import PolygonMesh
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

from mesh_splat.mesh_io import (
    decode_image,
    decode_mesh,
    image_suffix_for_array,
    stamp_to_ns,
    write_image_file,
    write_obj_mesh,
)


@dataclass
class PoseSample:
    timestamp_ns: int
    frame_id: str
    child_frame_id: str
    position_x: float
    position_y: float
    position_z: float
    orientation_w: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    linear_velocity_x: float
    linear_velocity_y: float
    linear_velocity_z: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float

    @classmethod
    def from_odometry(cls, msg: Odometry) -> "PoseSample":
        return cls(
            timestamp_ns=stamp_to_ns(msg.header.stamp),
            frame_id=msg.header.frame_id,
            child_frame_id=msg.child_frame_id,
            position_x=float(msg.pose.pose.position.x),
            position_y=float(msg.pose.pose.position.y),
            position_z=float(msg.pose.pose.position.z),
            orientation_w=float(msg.pose.pose.orientation.w),
            orientation_x=float(msg.pose.pose.orientation.x),
            orientation_y=float(msg.pose.pose.orientation.y),
            orientation_z=float(msg.pose.pose.orientation.z),
            linear_velocity_x=float(msg.twist.twist.linear.x),
            linear_velocity_y=float(msg.twist.twist.linear.y),
            linear_velocity_z=float(msg.twist.twist.linear.z),
            angular_velocity_x=float(msg.twist.twist.angular.x),
            angular_velocity_y=float(msg.twist.twist.angular.y),
            angular_velocity_z=float(msg.twist.twist.angular.z),
        )

    @classmethod
    def from_transform(
        cls,
        stamp,
        frame_id: str,
        child_frame_id: str,
        transform,
    ) -> "PoseSample":
        return cls(
            timestamp_ns=stamp_to_ns(stamp),
            frame_id=frame_id,
            child_frame_id=child_frame_id,
            position_x=float(transform.translation.x),
            position_y=float(transform.translation.y),
            position_z=float(transform.translation.z),
            orientation_w=float(transform.rotation.w),
            orientation_x=float(transform.rotation.x),
            orientation_y=float(transform.rotation.y),
            orientation_z=float(transform.rotation.z),
            linear_velocity_x=0.0,
            linear_velocity_y=0.0,
            linear_velocity_z=0.0,
            angular_velocity_x=0.0,
            angular_velocity_y=0.0,
            angular_velocity_z=0.0,
        )

    def as_row(self) -> dict[str, object]:
        return {
            "pose_timestamp_ns": self.timestamp_ns,
            "pose_frame_id": self.frame_id,
            "pose_child_frame_id": self.child_frame_id,
            "position_x": self.position_x,
            "position_y": self.position_y,
            "position_z": self.position_z,
            "orientation_w": self.orientation_w,
            "orientation_x": self.orientation_x,
            "orientation_y": self.orientation_y,
            "orientation_z": self.orientation_z,
            "linear_velocity_x": self.linear_velocity_x,
            "linear_velocity_y": self.linear_velocity_y,
            "linear_velocity_z": self.linear_velocity_z,
            "angular_velocity_x": self.angular_velocity_x,
            "angular_velocity_y": self.angular_velocity_y,
            "angular_velocity_z": self.angular_velocity_z,
        }


def _quaternion_to_matrix(
    qw: float, qx: float, qy: float, qz: float
) -> np.ndarray:
    norm = np.linalg.norm([qw, qx, qy, qz])
    if norm == 0.0:
        return np.eye(3, dtype=np.float64)

    qw, qx, qy, qz = (value / norm for value in (qw, qx, qy, qz))
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )


def _matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s

    norm = np.linalg.norm([qw, qx, qy, qz])
    if norm == 0.0:
        return 1.0, 0.0, 0.0, 0.0
    return tuple(float(value / norm) for value in (qw, qx, qy, qz))


def _pose_sample_to_matrix(pose: PoseSample) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _quaternion_to_matrix(
        pose.orientation_w,
        pose.orientation_x,
        pose.orientation_y,
        pose.orientation_z,
    )
    transform[:3, 3] = np.array(
        [pose.position_x, pose.position_y, pose.position_z], dtype=np.float64
    )
    return transform


class DatasetRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("dataset_recorder")

        self.declare_parameter("output_dir", "artifacts/kimera_dataset_capture")
        self.declare_parameter("left_image_topic", "/cam0/image_raw")
        self.declare_parameter("right_image_topic", "/cam1/image_raw")
        self.declare_parameter("odometry_topic", "/kimera_vio_ros/odometry")
        self.declare_parameter("mesh_topic", "/kimera_vio_ros/mesh")
        self.declare_parameter("texture_topic", "/kimera_vio_ros/debug_mesh_img")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("base_link_frame_id", "base_link")
        self.declare_parameter("params_folder", "")
        self.declare_parameter("left_camera_params_file", "LeftCameraParams.yaml")
        self.declare_parameter("right_camera_params_file", "RightCameraParams.yaml")
        self.declare_parameter("max_left_images", 0)
        self.declare_parameter("max_right_images", 0)
        self.declare_parameter("max_meshes", 0)
        self.declare_parameter("max_poses", 0)
        self.declare_parameter("require_pose_before_capture", False)
        self.declare_parameter("stop_when_complete", False)

        self.output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        ).expanduser()
        self.left_image_topic = (
            self.get_parameter("left_image_topic").get_parameter_value().string_value
        )
        self.right_image_topic = (
            self.get_parameter("right_image_topic").get_parameter_value().string_value
        )
        self.odometry_topic = (
            self.get_parameter("odometry_topic").get_parameter_value().string_value
        )
        self.mesh_topic = self.get_parameter("mesh_topic").get_parameter_value().string_value
        self.texture_topic = (
            self.get_parameter("texture_topic").get_parameter_value().string_value
        )
        self.tf_topic = self.get_parameter("tf_topic").get_parameter_value().string_value
        self.world_frame_id = (
            self.get_parameter("world_frame_id").get_parameter_value().string_value
        )
        self.base_link_frame_id = (
            self.get_parameter("base_link_frame_id").get_parameter_value().string_value
        )
        self.params_folder = Path(
            self.get_parameter("params_folder").get_parameter_value().string_value
        ).expanduser()
        self.left_camera_params_file = (
            self.get_parameter("left_camera_params_file")
            .get_parameter_value()
            .string_value
        )
        self.right_camera_params_file = (
            self.get_parameter("right_camera_params_file")
            .get_parameter_value()
            .string_value
        )
        self.max_left_images = (
            self.get_parameter("max_left_images").get_parameter_value().integer_value
        )
        self.max_right_images = (
            self.get_parameter("max_right_images").get_parameter_value().integer_value
        )
        self.max_meshes = (
            self.get_parameter("max_meshes").get_parameter_value().integer_value
        )
        self.max_poses = self.get_parameter("max_poses").get_parameter_value().integer_value
        self.require_pose_before_capture = (
            self.get_parameter("require_pose_before_capture")
            .get_parameter_value()
            .bool_value
        )
        self.stop_when_complete = (
            self.get_parameter("stop_when_complete").get_parameter_value().bool_value
        )

        self.images_left_dir = self.output_dir / "images" / "cam0"
        self.images_right_dir = self.output_dir / "images" / "cam1"
        self.meshes_dir = self.output_dir / "meshes"
        for directory in (
            self.output_dir,
            self.images_left_dir,
            self.images_right_dir,
            self.meshes_dir,
        ):
            directory.mkdir(parents=True, exist_ok=True)

        self._capture_started_ns = int(self.get_clock().now().nanoseconds)
        self._latest_pose: Optional[PoseSample] = None
        self._latest_texture = None
        self._latest_texture_stamp_ns: Optional[int] = None
        self._left_image_count = 0
        self._right_image_count = 0
        self._mesh_count = 0
        self._pose_count = 0
        self._closed = False
        self._camera_warned = False
        self._texture_warned = False
        self._mesh_warned = False
        self._waiting_for_pose_logged = False

        self._left_camera_frame_id = "cam0"
        self._right_camera_frame_id = "cam1"
        self._left_camera_pose_body: Optional[np.ndarray] = None
        self._right_camera_pose_body: Optional[np.ndarray] = None
        self._load_camera_extrinsics()

        self._metadata_path = self.output_dir / "metadata.json"
        self._write_metadata()

        self._odometry_handle = (self.output_dir / "odometry.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._odometry_writer = csv.DictWriter(
            self._odometry_handle,
            fieldnames=[
                "pose_timestamp_ns",
                "pose_frame_id",
                "pose_child_frame_id",
                "position_x",
                "position_y",
                "position_z",
                "orientation_w",
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "linear_velocity_x",
                "linear_velocity_y",
                "linear_velocity_z",
                "angular_velocity_x",
                "angular_velocity_y",
                "angular_velocity_z",
            ],
        )
        self._odometry_writer.writeheader()

        frame_fieldnames = [
            "image_timestamp_ns",
            "filename",
            "pose_timestamp_ns",
            "pose_frame_id",
            "pose_child_frame_id",
            "position_x",
            "position_y",
            "position_z",
            "orientation_w",
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "linear_velocity_x",
            "linear_velocity_y",
            "linear_velocity_z",
            "angular_velocity_x",
            "angular_velocity_y",
            "angular_velocity_z",
        ]
        self._cam0_handle = (self.output_dir / "cam0_frames.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._cam0_writer = csv.DictWriter(self._cam0_handle, fieldnames=frame_fieldnames)
        self._cam0_writer.writeheader()

        self._cam1_handle = (self.output_dir / "cam1_frames.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._cam1_writer = csv.DictWriter(self._cam1_handle, fieldnames=frame_fieldnames)
        self._cam1_writer.writeheader()

        self._meshes_handle = (self.output_dir / "meshes.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._meshes_writer = csv.DictWriter(
            self._meshes_handle,
            fieldnames=[
                "mesh_timestamp_ns",
                "obj_filename",
                "mtl_filename",
                "texture_filename",
                "texture_timestamp_ns",
                "vertex_count",
                "triangle_count",
                "pose_timestamp_ns",
                "pose_frame_id",
                "pose_child_frame_id",
                "position_x",
                "position_y",
                "position_z",
                "orientation_w",
                "orientation_x",
                "orientation_y",
                "orientation_z",
                "linear_velocity_x",
                "linear_velocity_y",
                "linear_velocity_z",
                "angular_velocity_x",
                "angular_velocity_y",
                "angular_velocity_z",
            ],
        )
        self._meshes_writer.writeheader()

        mesh_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Image,
            self.left_image_topic,
            self._left_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.right_image_topic,
            self._right_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.odometry_topic,
            self._odometry_callback,
            10,
        )
        self.create_subscription(
            TFMessage,
            self.tf_topic,
            self._tf_callback,
            50,
        )
        self.create_subscription(
            PolygonMesh,
            self.mesh_topic,
            self._mesh_callback,
            mesh_qos,
        )
        self.create_subscription(
            Image,
            self.texture_topic,
            self._texture_callback,
            mesh_qos,
        )

        self.get_logger().info(
            "Recording a small dataset into "
            f"'{self.output_dir}' from odometry='{self.odometry_topic}', "
            f"mesh='{self.mesh_topic}', cam0='{self.left_image_topic}', "
            f"cam1='{self.right_image_topic}'."
        )

    def _write_metadata(self) -> None:
        metadata = {
            "capture_started_ns": self._capture_started_ns,
            "output_dir": str(self.output_dir),
            "topics": {
                "left_image_topic": self.left_image_topic,
                "right_image_topic": self.right_image_topic,
                "odometry_topic": self.odometry_topic,
                "tf_topic": self.tf_topic,
                "mesh_topic": self.mesh_topic,
                "texture_topic": self.texture_topic,
            },
            "camera_frames": {
                "left_camera_frame_id": self._left_camera_frame_id,
                "right_camera_frame_id": self._right_camera_frame_id,
            },
            "limits": {
                "max_left_images": int(self.max_left_images),
                "max_right_images": int(self.max_right_images),
                "max_meshes": int(self.max_meshes),
                "max_poses": int(self.max_poses),
                "require_pose_before_capture": bool(self.require_pose_before_capture),
                "stop_when_complete": bool(self.stop_when_complete),
            },
            "counts": {
                "left_images": int(self._left_image_count),
                "right_images": int(self._right_image_count),
                "meshes": int(self._mesh_count),
                "poses": int(self._pose_count),
            },
            "pose_note": "Image and mesh pose rows use world -> base_link poses from odometry when available, otherwise from TF, and compose them with body -> camera extrinsics from the Kimera camera YAMLs when available.",
        }
        self._metadata_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._write_metadata()
        for handle in (
            self._odometry_handle,
            self._cam0_handle,
            self._cam1_handle,
            self._meshes_handle,
        ):
            with suppress(Exception):
                handle.flush()
            with suppress(Exception):
                handle.close()

    def _odometry_callback(self, msg: Odometry) -> None:
        if self.max_poses > 0 and self._pose_count >= self.max_poses:
            self._maybe_finish()
            return

        pose = PoseSample.from_odometry(msg)
        self._record_pose_sample(pose)

    def _tf_callback(self, msg: TFMessage) -> None:
        if self.max_poses > 0 and self._pose_count >= self.max_poses:
            self._maybe_finish()
            return

        for transform in msg.transforms:
            parent_frame = self._normalize_frame_id(transform.header.frame_id)
            child_frame = self._normalize_frame_id(transform.child_frame_id)
            if (
                parent_frame != self._normalize_frame_id(self.world_frame_id)
                or child_frame != self._normalize_frame_id(self.base_link_frame_id)
            ):
                continue

            pose = PoseSample.from_transform(
                transform.header.stamp,
                parent_frame,
                child_frame,
                transform.transform,
            )
            self._record_pose_sample(pose)
            break

    def _left_image_callback(self, msg: Image) -> None:
        self._record_image(
            msg=msg,
            output_dir=self.images_left_dir,
            writer=self._cam0_writer,
            handle=self._cam0_handle,
            topic_name=self.left_image_topic,
            frame_counter_name="_left_image_count",
            max_count=self.max_left_images,
            body_pose_sensor=self._left_camera_pose_body,
            sensor_frame_id=self._left_camera_frame_id,
        )

    def _right_image_callback(self, msg: Image) -> None:
        self._record_image(
            msg=msg,
            output_dir=self.images_right_dir,
            writer=self._cam1_writer,
            handle=self._cam1_handle,
            topic_name=self.right_image_topic,
            frame_counter_name="_right_image_count",
            max_count=self.max_right_images,
            body_pose_sensor=self._right_camera_pose_body,
            sensor_frame_id=self._right_camera_frame_id,
        )

    def _record_image(
        self,
        msg: Image,
        output_dir: Path,
        writer: csv.DictWriter,
        handle,
        topic_name: str,
        frame_counter_name: str,
        max_count: int,
        body_pose_sensor: Optional[np.ndarray],
        sensor_frame_id: str,
    ) -> None:
        current_count = getattr(self, frame_counter_name)
        if max_count > 0 and current_count >= max_count:
            self._maybe_finish()
            return
        if not self._ready_to_capture():
            return

        try:
            image = decode_image(msg)
        except ValueError as exc:
            if not self._camera_warned:
                self.get_logger().warning(str(exc))
                self._camera_warned = True
            return

        stamp_ns = stamp_to_ns(msg.header.stamp)
        filename = f"{stamp_ns}{image_suffix_for_array(image)}"
        write_image_file(output_dir / filename, image)

        row = {
            "image_timestamp_ns": stamp_ns,
            "filename": filename,
        }
        row.update(
            self._pose_row(
                self._latest_pose,
                body_pose_sensor=body_pose_sensor,
                sensor_frame_id=sensor_frame_id,
            )
        )
        writer.writerow(row)
        handle.flush()

        setattr(self, frame_counter_name, current_count + 1)
        if current_count == 0:
            self.get_logger().info(
                f"Saved first image from '{topic_name}' to '{output_dir / filename}'."
            )
        self._write_metadata()
        self._maybe_finish()

    def _texture_callback(self, msg: Image) -> None:
        try:
            self._latest_texture = decode_image(msg)
        except ValueError as exc:
            if not self._texture_warned:
                self.get_logger().warning(str(exc))
                self._texture_warned = True
            return
        self._latest_texture_stamp_ns = stamp_to_ns(msg.header.stamp)

    def _mesh_callback(self, msg: PolygonMesh) -> None:
        if self.max_meshes > 0 and self._mesh_count >= self.max_meshes:
            self._maybe_finish()
            return
        if not self._ready_to_capture():
            return

        try:
            mesh = decode_mesh(msg)
        except ValueError as exc:
            if not self._mesh_warned:
                self.get_logger().warning(str(exc))
                self._mesh_warned = True
            return

        stem = f"mesh_{self._mesh_count:04d}_{mesh.stamp_ns}"
        texture_filename = None
        if self._latest_texture is not None:
            texture_filename = f"{stem}_texture{image_suffix_for_array(self._latest_texture)}"
            write_image_file(self.meshes_dir / texture_filename, self._latest_texture)

        obj_filename = f"{stem}.obj"
        mtl_path = write_obj_mesh(
            self.meshes_dir / obj_filename,
            mesh,
            texture_filename=texture_filename,
        )

        row = {
            "mesh_timestamp_ns": mesh.stamp_ns,
            "obj_filename": obj_filename,
            "mtl_filename": mtl_path.name if mtl_path is not None else "",
            "texture_filename": texture_filename or "",
            "texture_timestamp_ns": self._latest_texture_stamp_ns or "",
            "vertex_count": int(mesh.vertex_positions.shape[0]),
            "triangle_count": int(mesh.triangle_indices.shape[0]),
        }
        row.update(
            self._pose_row(
                self._latest_pose,
                body_pose_sensor=self._left_camera_pose_body,
                sensor_frame_id=self._left_camera_frame_id,
            )
        )
        self._meshes_writer.writerow(row)
        self._meshes_handle.flush()

        self._mesh_count += 1
        if self._mesh_count == 1:
            self.get_logger().info(
                f"Saved first mesh snapshot to '{self.meshes_dir / obj_filename}'."
            )
        self._write_metadata()
        self._maybe_finish()

    def _pose_row(
        self,
        pose: Optional[PoseSample],
        body_pose_sensor: Optional[np.ndarray] = None,
        sensor_frame_id: str = "",
    ) -> dict[str, object]:
        if pose is None:
            return {
                "pose_timestamp_ns": "",
                "pose_frame_id": "",
                "pose_child_frame_id": "",
                "position_x": "",
                "position_y": "",
                "position_z": "",
                "orientation_w": "",
                "orientation_x": "",
                "orientation_y": "",
                "orientation_z": "",
                "linear_velocity_x": "",
                "linear_velocity_y": "",
                "linear_velocity_z": "",
                "angular_velocity_x": "",
                "angular_velocity_y": "",
                "angular_velocity_z": "",
            }
        if body_pose_sensor is None:
            return pose.as_row()

        world_pose_sensor = _pose_sample_to_matrix(pose) @ body_pose_sensor
        orientation_w, orientation_x, orientation_y, orientation_z = (
            _matrix_to_quaternion(world_pose_sensor[:3, :3])
        )
        return {
            "pose_timestamp_ns": pose.timestamp_ns,
            "pose_frame_id": pose.frame_id,
            "pose_child_frame_id": sensor_frame_id or pose.child_frame_id,
            "position_x": float(world_pose_sensor[0, 3]),
            "position_y": float(world_pose_sensor[1, 3]),
            "position_z": float(world_pose_sensor[2, 3]),
            "orientation_w": orientation_w,
            "orientation_x": orientation_x,
            "orientation_y": orientation_y,
            "orientation_z": orientation_z,
            "linear_velocity_x": "",
            "linear_velocity_y": "",
            "linear_velocity_z": "",
            "angular_velocity_x": "",
            "angular_velocity_y": "",
            "angular_velocity_z": "",
        }

    def _load_camera_extrinsics(self) -> None:
        if not self.params_folder or not self.params_folder.exists():
            self.get_logger().warning(
                "params_folder was not set or does not exist; image pose rows will fall back to raw odometry poses."
            )
            return

        left_result = self._read_camera_extrinsic(
            self.params_folder / self.left_camera_params_file
        )
        right_result = self._read_camera_extrinsic(
            self.params_folder / self.right_camera_params_file
        )
        if left_result is not None:
            self._left_camera_frame_id, self._left_camera_pose_body = left_result
        if right_result is not None:
            self._right_camera_frame_id, self._right_camera_pose_body = right_result

    def _read_camera_extrinsic(
        self, camera_params_path: Path
    ) -> Optional[tuple[str, np.ndarray]]:
        if not camera_params_path.exists():
            self.get_logger().warning(
                f"Camera params file '{camera_params_path}' does not exist; falling back to raw odometry poses."
            )
            return None

        contents = camera_params_path.read_text(encoding="utf-8")
        camera_id_match = re.search(r"camera_id:\s*([^\s]+)", contents)
        matrix_match = re.search(r"T_BS:\s*.*?data:\s*\[([^\]]+)\]", contents, re.DOTALL)
        if camera_id_match is None or matrix_match is None:
            self.get_logger().warning(
                f"Could not parse camera extrinsics from '{camera_params_path}'; falling back to raw odometry poses."
            )
            return None

        values = [
            float(token.strip())
            for token in matrix_match.group(1).replace("\n", " ").split(",")
            if token.strip()
        ]
        if len(values) != 16:
            self.get_logger().warning(
                f"Expected 16 values in T_BS for '{camera_params_path}', found {len(values)}."
            )
            return None

        return (
            camera_id_match.group(1),
            np.asarray(values, dtype=np.float64).reshape((4, 4)),
        )

    def _record_pose_sample(self, pose: PoseSample) -> None:
        self._latest_pose = pose
        self._odometry_writer.writerow(pose.as_row())
        self._odometry_handle.flush()
        self._pose_count += 1
        self._write_metadata()
        self._maybe_finish()

    def _ready_to_capture(self) -> bool:
        if not self.require_pose_before_capture or self._latest_pose is not None:
            return True
        if not self._waiting_for_pose_logged:
            self.get_logger().info(
                "Waiting for first world->base_link pose before saving frames."
            )
            self._waiting_for_pose_logged = True
        return False

    @staticmethod
    def _normalize_frame_id(frame_id: str) -> str:
        return frame_id[1:] if frame_id.startswith("/") else frame_id

    def _maybe_finish(self) -> None:
        if not self.stop_when_complete:
            return

        completion_states = []
        if self.max_left_images > 0:
            completion_states.append(self._left_image_count >= self.max_left_images)
        if self.max_right_images > 0:
            completion_states.append(self._right_image_count >= self.max_right_images)
        if self.max_meshes > 0:
            completion_states.append(self._mesh_count >= self.max_meshes)
        if self.max_poses > 0:
            completion_states.append(self._pose_count >= self.max_poses)

        if completion_states and all(completion_states):
            self.get_logger().info(
                "Dataset capture limits reached; shutting down recorder."
            )
            self.close()
            rclpy.shutdown()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[DatasetRecorderNode] = None
    try:
        node = DatasetRecorderNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        if node is not None:
            with suppress(Exception, KeyboardInterrupt):
                node.close()
            with suppress(Exception, KeyboardInterrupt):
                node.destroy_node()
        with suppress(Exception, KeyboardInterrupt):
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
