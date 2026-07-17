from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
    return f'graco-aerial-05-da3-conditioned-offline-{timestamp}'


def _validate(context):
    bag_path = Path(LaunchConfiguration('input_bag_path').perform(context))
    if not bag_path.is_dir() or not (bag_path / 'metadata.yaml').is_file():
        raise RuntimeError(
            'input_bag_path must name a finalized rosbag directory'
        )
    for argument in ('engine_path', 'camera_calibration_path'):
        path = Path(LaunchConfiguration(argument).perform(context))
        if not path.is_file():
            raise RuntimeError(f'{argument} must name an existing file')

    positive_floats = (
        'playback_rate',
        'playback_delay_s',
        'inference_drain_delay_s',
        'image_cache.duration_s',
    )
    for argument in positive_floats:
        if float(LaunchConfiguration(argument).perform(context)) <= 0.0:
            raise RuntimeError(f'{argument} must be positive')
    if int(
        LaunchConfiguration('read_ahead_queue_size').perform(context)
    ) <= 0:
        raise RuntimeError('read_ahead_queue_size must be positive')
    return []


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('input_bag_path'),
        DeclareLaunchArgument('playback_rate', default_value='5.0'),
        DeclareLaunchArgument('playback_delay_s', default_value='3.0'),
        DeclareLaunchArgument(
            'inference_drain_delay_s', default_value='30.0'),
        DeclareLaunchArgument(
            'read_ahead_queue_size', default_value='500'),
        DeclareLaunchArgument(
            'image_cache.duration_s', default_value='15.0'),
        DeclareLaunchArgument(
            'inference.maximum_pending_runs', default_value='8'),
        DeclareLaunchArgument(
            'submap_sparse_ba.enabled', default_value='true'),
        DeclareLaunchArgument(
            'submap_sparse_ba.global.enabled', default_value='true'),
        DeclareLaunchArgument(
            'submap_sparse_ba.pose_initialization_source',
            default_value='first_estimate'),
        DeclareLaunchArgument(
            'submap_sparse_ba.idle_optimization_delay_s',
            default_value='8.0'),
        DeclareLaunchArgument(
            'submap_sparse_ba.minimum_idle_run_count', default_value='1'),
        DeclareLaunchArgument('engine_path', default_value=(
            POSE_CONDITIONED_DA3_ENGINE
        )),
        DeclareLaunchArgument(
            'camera_calibration_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('kimera_vio_ros'),
                'param',
                'GrAcoMonoXfeat',
                'LeftCameraParams.yaml',
            ])),
        DeclareLaunchArgument('rerun.enabled', default_value='true'),
        DeclareLaunchArgument(
            'rerun.host',
            default_value='rerun+http://127.0.0.1:9876/proxy'),
        DeclareLaunchArgument(
            'rerun.application_id',
            default_value='graco_aerial_05_dense_mapping_experiment'),
        DeclareLaunchArgument(
            'rerun.recording_id',
            default_value=_timestamped_recording_id()),
        DeclareLaunchArgument(
            'rerun.entity_prefix',
            default_value=(
                'a5/dense_mapping_experiment/odometry_conditioned_offline'
            )),
    ]

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
            'image_cache.duration_s': LaunchConfiguration(
                'image_cache.duration_s'),
            'raw_image_qos.reliability': 'reliable',
            'raw_image_qos.depth': '200',
            'inference.maximum_pending_runs': LaunchConfiguration(
                'inference.maximum_pending_runs'),
            'point_stride': '4',
            'max_points_per_view': '100000',
            'max_points_per_submap': '200000',
            'max_runs_per_submap': '5',
            'min_depth_m': '0.1',
            'max_depth_m': '100.0',
            'geometry_filter.max_relative_depth_error': '0.15',
            'geometry_filter.visualization_max_relative_error': '0.5',
            'rerun.enabled': LaunchConfiguration('rerun.enabled'),
            'rerun.application_id': LaunchConfiguration(
                'rerun.application_id'),
            'rerun.recording_id': LaunchConfiguration(
                'rerun.recording_id'),
            'rerun.host': LaunchConfiguration('rerun.host'),
            'rerun.entity_prefix': LaunchConfiguration(
                'rerun.entity_prefix'),
            'rerun.point_radius': '1.0',
            'input_bag_record.enabled': 'false',
            'submap_sparse_ba.enabled': LaunchConfiguration(
                'submap_sparse_ba.enabled'),
            'submap_sparse_ba.global.enabled': LaunchConfiguration(
                'submap_sparse_ba.global.enabled'),
            'submap_sparse_ba.pose_initialization_source': (
                LaunchConfiguration(
                    'submap_sparse_ba.pose_initialization_source')
            ),
            'submap_sparse_ba.idle_optimization_delay_s': LaunchConfiguration(
                'submap_sparse_ba.idle_optimization_delay_s'),
            'submap_sparse_ba.minimum_idle_run_count': LaunchConfiguration(
                'submap_sparse_ba.minimum_idle_run_count'),
        }.items(),
    )

    player = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            LaunchConfiguration('input_bag_path'),
            '--rate',
            LaunchConfiguration('playback_rate'),
            '--start-paused',
            '--read-ahead-queue-size',
            LaunchConfiguration('read_ahead_queue_size'),
            '--disable-keyboard-controls',
            '--wait-for-all-acked',
            '10000',
        ],
        output='screen',
    )
    resume_player = TimerAction(
        period=LaunchConfiguration('playback_delay_s'),
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/rosbag2_player/resume',
                    'rosbag2_interfaces/srv/Resume',
                    '{}',
                ],
                output='screen',
            ),
        ],
    )
    shutdown_after_drain = RegisterEventHandler(
        OnProcessExit(
            target_action=player,
            on_exit=[
                TimerAction(
                    period=LaunchConfiguration('inference_drain_delay_s'),
                    actions=[
                        EmitEvent(event=Shutdown(
                            reason=(
                                'offline rosbag finished and inference '
                                'queue drained'
                            )
                        )),
                    ],
                ),
            ],
        )
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=_validate),
        experiment,
        player,
        resume_player,
        shutdown_after_drain,
    ])
