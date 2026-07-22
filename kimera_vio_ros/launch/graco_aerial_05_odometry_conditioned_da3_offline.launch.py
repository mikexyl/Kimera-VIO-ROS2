from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare

from dense_mapping_launch import process_exit_handler


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%z')
    return f'graco-aerial-05-da3-conditioned-offline-{timestamp}'


def _boolean_argument(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    raise RuntimeError(f'{name} must be a boolean')


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
        'playback_duration_s',
        'inference_drain_delay_s',
        'image_cache.duration_s',
        'zenoh_router_startup_delay_s',
        'geometry_filter.minimum_disparity_px',
        'geometry_filter.visualization_max_disparity_px',
    )
    for argument in positive_floats:
        if float(LaunchConfiguration(argument).perform(context)) <= 0.0:
            raise RuntimeError(f'{argument} must be positive')
    for argument in (
        'read_ahead_queue_size',
        'output_qos.depth',
        'max_runs_per_submap',
    ):
        if int(LaunchConfiguration(argument).perform(context)) <= 0:
            raise RuntimeError(f'{argument} must be positive')
    ba_enabled = _boolean_argument(context, 'submap_sparse_ba.enabled')
    global_enabled = _boolean_argument(
        context, 'submap_sparse_ba.global.enabled')
    depth_refiner_enabled = _boolean_argument(
        context, 'submap_sparse_ba.depth_refiner.enabled')
    if global_enabled and not ba_enabled:
        raise RuntimeError(
            'submap_sparse_ba.global.enabled requires '
            'submap_sparse_ba.enabled')
    if depth_refiner_enabled and not global_enabled:
        raise RuntimeError(
            'submap_sparse_ba.depth_refiner.enabled requires '
            'submap_sparse_ba.global.enabled')
    return []


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('input_bag_path'),
        DeclareLaunchArgument('start_zenoh_router', default_value='true'),
        DeclareLaunchArgument(
            'zenoh_router_startup_delay_s', default_value='1.0'),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        DeclareLaunchArgument('playback_delay_s', default_value='3.0'),
        DeclareLaunchArgument('playback_duration_s', default_value='118.0'),
        DeclareLaunchArgument(
            'inference_drain_delay_s', default_value='60.0'),
        DeclareLaunchArgument(
            'read_ahead_queue_size', default_value='500'),
        DeclareLaunchArgument('output_qos.depth', default_value='1000'),
        DeclareLaunchArgument('max_runs_per_submap', default_value='1'),
        DeclareLaunchArgument(
            'geometry_filter.minimum_disparity_px', default_value='100.0'),
        DeclareLaunchArgument(
            'geometry_filter.visualization_max_disparity_px',
            default_value='100.0'),
        DeclareLaunchArgument(
            'image_cache.duration_s', default_value='15.0'),
        DeclareLaunchArgument(
            'inference.maximum_pending_runs', default_value='8'),
        DeclareLaunchArgument(
            'submap_sparse_ba.enabled', default_value='true'),
        DeclareLaunchArgument(
            'submap_sparse_ba.global.enabled', default_value='true'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.enabled', default_value='true'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.grid_rows', default_value='4'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.grid_cols', default_value='4'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.'
            'sparse_landmark_constraints.enabled', default_value='true'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.landmark_support_filter.enabled',
            default_value='true'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.landmark_support_filter.radius_px',
            default_value='128'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.landmark_support_filter.'
            'minimum_landmarks', default_value='3'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.two_view_consistency.enabled',
            default_value='false'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.two_view_consistency.sample_stride',
            default_value='16'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.two_view_consistency.'
            'maximum_constraints', default_value='2000'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.two_view_consistency.'
            'measurement_sigma', default_value='0.20'),
        DeclareLaunchArgument(
            'submap_sparse_ba.depth_refiner.two_view_consistency.'
            'max_relative_depth_error', default_value='0.25'),
        DeclareLaunchArgument(
            'submap_sparse_ba.pose_initialization_source',
            default_value='first_estimate'),
        DeclareLaunchArgument(
            'submap_sparse_ba.idle_optimization_delay_s',
            default_value='8.0'),
        DeclareLaunchArgument(
            'submap_sparse_ba.minimum_idle_run_count', default_value='1'),
        DeclareLaunchArgument(
            'comparison.da3_chain.enabled', default_value='true'),
        DeclareLaunchArgument('engine_path'),
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
            'output_qos.depth': LaunchConfiguration('output_qos.depth'),
            'inference.maximum_pending_runs': LaunchConfiguration(
                'inference.maximum_pending_runs'),
            'point_stride': '4',
            'max_points_per_view': '100000',
            'max_points_per_submap': '200000',
            'max_runs_per_submap': LaunchConfiguration(
                'max_runs_per_submap'),
            'min_depth_m': '0.1',
            'max_depth_m': '100.0',
            'geometry_filter.max_relative_depth_error': '0.15',
            'geometry_filter.visualization_max_relative_error': '0.5',
            'geometry_filter.minimum_disparity_px': LaunchConfiguration(
                'geometry_filter.minimum_disparity_px'),
            'geometry_filter.visualization_max_disparity_px': (
                LaunchConfiguration(
                    'geometry_filter.visualization_max_disparity_px')
            ),
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
            'submap_sparse_ba.depth_refiner.enabled': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.enabled'),
            'submap_sparse_ba.depth_refiner.grid_rows': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.grid_rows'),
            'submap_sparse_ba.depth_refiner.grid_cols': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.grid_cols'),
            'submap_sparse_ba.depth_refiner.'
            'sparse_landmark_constraints.enabled': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.'
                'sparse_landmark_constraints.enabled'),
            'submap_sparse_ba.depth_refiner.landmark_support_filter.enabled': (
                LaunchConfiguration(
                    'submap_sparse_ba.depth_refiner.'
                    'landmark_support_filter.enabled')
            ),
            'submap_sparse_ba.depth_refiner.landmark_support_filter.'
            'radius_px': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.'
                'landmark_support_filter.radius_px'),
            'submap_sparse_ba.depth_refiner.landmark_support_filter.'
            'minimum_landmarks': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.'
                'landmark_support_filter.minimum_landmarks'),
            'submap_sparse_ba.depth_refiner.two_view_consistency.enabled': (
                LaunchConfiguration(
                    'submap_sparse_ba.depth_refiner.two_view_consistency.'
                    'enabled')
            ),
            'submap_sparse_ba.depth_refiner.two_view_consistency.'
            'sample_stride': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.two_view_consistency.'
                'sample_stride'),
            'submap_sparse_ba.depth_refiner.two_view_consistency.'
            'maximum_constraints': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.two_view_consistency.'
                'maximum_constraints'),
            'submap_sparse_ba.depth_refiner.two_view_consistency.'
            'measurement_sigma': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.two_view_consistency.'
                'measurement_sigma'),
            'submap_sparse_ba.depth_refiner.two_view_consistency.'
            'max_relative_depth_error': LaunchConfiguration(
                'submap_sparse_ba.depth_refiner.two_view_consistency.'
                'max_relative_depth_error'),
            'submap_sparse_ba.pose_initialization_source': (
                LaunchConfiguration(
                    'submap_sparse_ba.pose_initialization_source')
            ),
            'submap_sparse_ba.idle_optimization_delay_s': LaunchConfiguration(
                'submap_sparse_ba.idle_optimization_delay_s'),
            'submap_sparse_ba.minimum_idle_run_count': LaunchConfiguration(
                'submap_sparse_ba.minimum_idle_run_count'),
            'comparison.da3_chain.enabled': LaunchConfiguration(
                'comparison.da3_chain.enabled'),
        }.items(),
    )

    player = ExecuteProcess(
        cmd=[
            'timeout',
            '--signal=INT',
            PythonExpression([
                'str(float(',
                LaunchConfiguration('playback_delay_s'),
                ') + float(',
                LaunchConfiguration('playback_duration_s'),
                ') / float(',
                LaunchConfiguration('playback_rate'),
                ') + 1.0)',
            ]),
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
    request_final_global_ba = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/a5/dense_mapping_experiment/odometry_conditioned/'
            'request_global_ba',
            'std_srvs/srv/Trigger',
            '{}',
        ],
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('submap_sparse_ba.global.enabled')),
    )
    resume_request = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/rosbag2_player/resume',
            'rosbag2_interfaces/srv/Resume',
            '{}',
        ],
        output='screen',
    )
    resume_player = TimerAction(
        period=LaunchConfiguration('playback_delay_s'),
        actions=[resume_request],
    )
    drain_shutdown = TimerAction(
        period=LaunchConfiguration('inference_drain_delay_s'),
        actions=[EmitEvent(event=Shutdown(
            reason='offline rosbag finished and inference queue drained'
        ))],
    )
    player_exit = process_exit_handler(
        player,
        'offline rosbag player',
        expected_return_codes=(0, 124),
        on_expected=(request_final_global_ba, drain_shutdown),
    )
    final_ba_request_exit = process_exit_handler(
        request_final_global_ba,
        'final global BA service request',
        expected_return_codes=(0,),
    )
    resume_request_exit = process_exit_handler(
        resume_request,
        'rosbag resume service request',
        expected_return_codes=(0,),
    )
    zenoh_router = ExecuteProcess(
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_zenoh_router')),
        on_exit=EmitEvent(event=Shutdown(
            reason='ROS 2 Zenoh router exited')),
    )
    pipeline = TimerAction(
        period=LaunchConfiguration('zenoh_router_startup_delay_s'),
        actions=[experiment, player, resume_player],
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=_validate),
        zenoh_router,
        player_exit,
        final_ba_request_exit,
        resume_request_exit,
        pipeline,
    ])
