from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


POSE_CONDITIONED_DA3_ENGINE = (
    '/home/mikexyl/workspaces/xfeat_cpp_ws/xfeat-cpp/onnx_model/'
    'mono_depth/depth_anything_v3/'
    'DA3-LARGE-1.1_pose_v2_350x504_fp16.engine'
)


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%z')
    return f'graco-aerial-05-da3-odometry-conditioned-{timestamp}'


def _timestamped_input_bag_path():
    timestamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%z')
    return f'/data/graco/odometry-conditioned-da3-inputs-{timestamp}'


def _validate_files(context):
    for argument in ('engine_path', 'camera_calibration_path'):
        path = Path(LaunchConfiguration(argument).perform(context))
        if not path.is_file():
            raise RuntimeError(f'{argument} must name an existing file')
    return []


def generate_launch_description():
    rerun_application_id = LaunchConfiguration('rerun_application_id')
    rerun_recording_id = LaunchConfiguration('rerun_recording_id')
    rerun_host = LaunchConfiguration('rerun_host')

    vio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('kimera_vio_ros'),
                'launch',
                'kimera_vio_ros_mono.launch.py',
            ])
        ),
        launch_arguments={
            'dataset_name': 'GrAcoMonoXfeat',
            'robot_name': 'a5',
            'robot_namespace': 'a5',
            'frame_id.base_link': 'a5/base_link',
            'frame_id.odom': 'a5/odom',
            'frame_id.map': 'a5/map',
            'frame_id.world': 'world',
            'topic.image': '/camera_left/image_raw',
            'topic.imu.data': '/gnss/imu',
            'topic.camera.info': '/camera_left/camera_info',
            'use_camera_info': 'false',
            'use_lcd': '0',
            'multi_robot_bridge.enabled': 'false',
            'use_sim_time': 'true',
            'start_zenoh_router': LaunchConfiguration(
                'start_zenoh_router'),
            'rerun_host': rerun_host,
            'rerun_application_id': rerun_application_id,
            'rerun_recording_id': rerun_recording_id,
            'use_rerun_visualizer': 'true',
            'rosbag_play': 'true',
            'rosbag_path': LaunchConfiguration('rosbag_path'),
            'rosbag_source_image_topic': '/camera_left/image_raw',
            'rosbag_source_imu_topic': '/gnss/imu',
            'rosbag_play_delay': '4.0',
            'rosbag_rate': LaunchConfiguration('rosbag_rate'),
            'rosbag_play_duration': LaunchConfiguration(
                'rosbag_play_duration'),
            'dense_mapping.publisher_enabled': 'true',
            'mono_depth.enabled': 'false',
            'mono_depth.da3_essential_factors_enabled': 'false',
            'mono_depth.da3_baseline_ratio_factors_enabled': 'false',
        }.items(),
    )

    experiment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dense_mapping'),
                'launch',
                'odometry_conditioned_da3.launch.py',
            ])
        ),
        launch_arguments={
            'robot': 'a5',
            'frame_id.map': 'a5/map',
            'frame_id.odometry': 'world',
            'topics.raw_image': '/camera_left/image_raw',
            'topics.keyframes': '/a5/kimera_vio/mapping/keyframes',
            'topics.local_window_poses': (
                '/a5/kimera_vio/mapping/local_window_poses'
            ),
            'camera_calibration_path': LaunchConfiguration(
                'camera_calibration_path'),
            'engine_path': LaunchConfiguration('engine_path'),
            'selection.minimum_distance_m': '10.0',
            'minimum_confidence': '1.2',
            'image_cache.duration_s': '3.0',
            'inference.maximum_pending_runs': '2',
            'point_stride': '4',
            'max_points_per_view': '100000',
            'max_points_per_submap': '200000',
            'max_runs_per_submap': '5',
            'min_depth_m': '0.1',
            'max_depth_m': '100.0',
            'geometry_filter.max_relative_depth_error': '0.15',
            'geometry_filter.visualization_max_relative_error': '0.5',
            'rerun.enabled': 'true',
            'rerun.application_id': rerun_application_id,
            'rerun.recording_id': rerun_recording_id,
            'rerun.host': rerun_host,
            'rerun.entity_prefix': (
                'a5/dense_mapping_experiment/odometry_conditioned'
            ),
            'rerun.point_radius': '1.0',
            'input_bag_record.enabled': LaunchConfiguration(
                'record_experiment_inputs'),
            'input_bag_record.output_directory': LaunchConfiguration(
                'experiment_input_bag_path'),
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', '/tmp/ros_logs'),
        DeclareLaunchArgument(
            'rosbag_path', default_value='/data/graco/aerial-05-40m'),
        DeclareLaunchArgument('rosbag_play_duration', default_value='0.0'),
        DeclareLaunchArgument('rosbag_rate', default_value='1.0'),
        DeclareLaunchArgument('start_zenoh_router', default_value='true'),
        DeclareLaunchArgument(
            'record_experiment_inputs', default_value='true'),
        DeclareLaunchArgument(
            'experiment_input_bag_path',
            default_value=_timestamped_input_bag_path()),
        DeclareLaunchArgument(
            'engine_path', default_value=POSE_CONDITIONED_DA3_ENGINE),
        DeclareLaunchArgument(
            'camera_calibration_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('kimera_vio_ros'),
                'param',
                'GrAcoMonoXfeat',
                'LeftCameraParams.yaml',
            ])),
        DeclareLaunchArgument(
            'rerun_host',
            default_value='rerun+http://127.0.0.1:9876/proxy'),
        DeclareLaunchArgument(
            'rerun_application_id',
            default_value='graco_aerial_05_dense_mapping_experiment'),
        DeclareLaunchArgument(
            'rerun_recording_id',
            default_value=_timestamped_recording_id()),
        OpaqueFunction(function=_validate_files),
        experiment,
        TimerAction(period=1.0, actions=[vio]),
    ])
