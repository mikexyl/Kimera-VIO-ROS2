from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


DA3_TWO_VIEW_ENGINE = (
    '/home/mikexyl/workspaces/sb_slam_ros2_ws/src/xfeat-cpp/onnx_model/'
    'DA3METRIC-LARGE_280x504_fp16.engine'
)


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%z')
    return f'graco-aerial-05-da3-two-view-{timestamp}'


def generate_launch_description():
    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value='/data/graco/aerial-05-40m',
        description='Converted ROS2 GrAco aerial-05 bag directory.',
    )
    rosbag_play_duration_arg = DeclareLaunchArgument(
        'rosbag_play_duration',
        default_value='0.0',
        description=(
            'Wall-clock seconds to run after bag playback starts; '
            '0 runs until manually stopped.'
        ),
    )
    rosbag_rate_arg = DeclareLaunchArgument(
        'rosbag_rate',
        default_value='1.0',
        description='ROS bag playback rate.',
    )
    start_zenoh_router_arg = DeclareLaunchArgument(
        'start_zenoh_router',
        default_value='true',
        description='Start rmw_zenohd; set false when a router is already running.',
    )
    rerun_host_arg = DeclareLaunchArgument(
        'rerun_host',
        default_value='rerun+http://127.0.0.1:9876/proxy',
        description='Rerun endpoint.',
    )
    rerun_application_id_arg = DeclareLaunchArgument(
        'rerun_application_id',
        default_value='graco_aerial_05_dense_mapping',
    )
    rerun_recording_id_arg = DeclareLaunchArgument(
        'rerun_recording_id',
        default_value=_timestamped_recording_id(),
    )
    geometry_filter_enabled_arg = DeclareLaunchArgument(
        'dense_mapping.geometry_filter_enabled',
        default_value='true',
    )
    submap_metric_scale_method_arg = DeclareLaunchArgument(
        'dense_mapping.submap_metric_scale_method',
        default_value='odometry',
        description='Submap metric scaling: none or odometry.',
    )
    submap_anchor_method_arg = DeclareLaunchArgument(
        'dense_mapping.submap_anchor_method',
        default_value='odometry',
        description='Submap anchoring: da3 or odometry.',
    )
    geometry_filter_pose_source_arg = DeclareLaunchArgument(
        'dense_mapping.geometry_filter_pose_source',
        default_value='da3',
        description='Depth reprojection pose source: da3 or odometry.',
    )
    geometry_filter_max_error_arg = DeclareLaunchArgument(
        'dense_mapping.geometry_filter_max_relative_depth_error',
        default_value='0.15',
    )
    geometry_filter_visualization_max_error_arg = DeclareLaunchArgument(
        'dense_mapping.geometry_filter_visualization_max_relative_error',
        default_value='0.5',
    )
    mono_depth_engine_arg = DeclareLaunchArgument(
        'mono_depth.engine_path',
        default_value=DA3_TWO_VIEW_ENGINE,
        description='Pose-free two-view DA3 TensorRT engine.',
    )
    mono_depth_da3_keyframe_selection_method_arg = DeclareLaunchArgument(
        'mono_depth.da3_keyframe_selection_method',
        default_value='distance',
        description='DA3 two-view endpoint selection: distance, fixed_skip, or covisibility.',
    )
    mono_depth_da3_keyframe_skip_arg = DeclareLaunchArgument(
        'mono_depth.da3_keyframe_skip',
        default_value='0',
        description='VIO keyframes held between DA3 endpoints in fixed_skip mode.',
    )
    mono_depth_da3_keyframe_covisibility_threshold_arg = DeclareLaunchArgument(
        'mono_depth.da3_keyframe_covisibility_threshold',
        default_value='0.3',
        description='Run DA3 when reference-track covisibility falls below this fraction.',
    )
    mono_depth_min_distance_arg = DeclareLaunchArgument(
        'mono_depth.min_keyframe_distance_m',
        default_value='10.0',
        description='Minimum odometry camera-center displacement for DA3.',
    )
    mono_depth_min_confidence_arg = DeclareLaunchArgument(
        'mono_depth.min_confidence',
        default_value='1.2',
        description='Minimum DA3 multi-view confidence.',
    )
    mono_depth_scale_alignment_method_arg = DeclareLaunchArgument(
        'mono_depth.scale_alignment_method',
        default_value='landmarks',
        description=(
            'Mono-depth scale alignment: relative_pose, landmarks, or none.'
        ),
    )
    mono_depth_da3_essential_factors_enabled_arg = DeclareLaunchArgument(
        'mono_depth.da3_essential_factors_enabled',
        default_value='false',
        description=(
            'Add DA3 essential-matrix factors to the fixed-lag smoother.'
        ),
    )
    mono_depth_da3_baseline_ratio_factors_enabled_arg = DeclareLaunchArgument(
        'mono_depth.da3_baseline_ratio_factors_enabled',
        default_value='false',
        description=(
            'Add scale-free consecutive DA3 baseline-ratio factors to the smoother.'
        ),
    )
    mono_depth_da3_baseline_ratio_log_sigma_arg = DeclareLaunchArgument(
        'mono_depth.da3_baseline_ratio_log_sigma',
        default_value='0.25',
        description='Standard deviation of the DA3 log baseline-ratio residual.',
    )
    mono_depth_landmark_scale_flatness_radius_arg = DeclareLaunchArgument(
        'mono_depth.landmark_scale_flatness_radius',
        default_value='4',
        description='Pixel radius used to detect depth edges around scale landmarks.',
    )
    mono_depth_landmark_scale_max_relative_depth_variation_arg = (
        DeclareLaunchArgument(
            'mono_depth.landmark_scale_max_relative_depth_variation',
            default_value='0.15',
            description=(
                'Maximum local relative depth variation before rejecting a '
                'scale landmark.'
            ),
        )
    )

    mono_launch = IncludeLaunchDescription(
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
            'use_sim_time': 'true',
            'start_zenoh_router': LaunchConfiguration('start_zenoh_router'),
            'rerun_host': LaunchConfiguration('rerun_host'),
            'rerun_application_id': LaunchConfiguration(
                'rerun_application_id'
            ),
            'rerun_recording_id': LaunchConfiguration('rerun_recording_id'),
            'rosbag_play': 'true',
            'rosbag_path': LaunchConfiguration('rosbag_path'),
            'rosbag_source_image_topic': '/camera_left/image_raw',
            'rosbag_source_imu_topic': '/gnss/imu',
            'rosbag_play_delay': '4.0',
            'rosbag_rate': LaunchConfiguration('rosbag_rate'),
            'rosbag_play_duration': LaunchConfiguration(
                'rosbag_play_duration'
            ),
            'mono_depth.enabled': 'true',
            'dense_mapping.publisher_enabled': 'true',
            'mono_depth.engine_path': LaunchConfiguration(
                'mono_depth.engine_path'
            ),
            'mono_depth.mode': 'multi_view',
            'mono_depth.da3_keyframe_selection_method': LaunchConfiguration(
                'mono_depth.da3_keyframe_selection_method'
            ),
            'mono_depth.da3_keyframe_skip': LaunchConfiguration(
                'mono_depth.da3_keyframe_skip'
            ),
            'mono_depth.da3_keyframe_covisibility_threshold': (
                LaunchConfiguration(
                    'mono_depth.da3_keyframe_covisibility_threshold'
                )
            ),
            'mono_depth.min_keyframe_distance_m': LaunchConfiguration(
                'mono_depth.min_keyframe_distance_m'
            ),
            'mono_depth.min_confidence': LaunchConfiguration(
                'mono_depth.min_confidence'
            ),
            'mono_depth.visualize_confidence': 'false',
            'mono_depth.scale_alignment_method': LaunchConfiguration(
                'mono_depth.scale_alignment_method'
            ),
            'mono_depth.da3_essential_factors_enabled': LaunchConfiguration(
                'mono_depth.da3_essential_factors_enabled'
            ),
            'mono_depth.da3_baseline_ratio_factors_enabled': (
                LaunchConfiguration(
                    'mono_depth.da3_baseline_ratio_factors_enabled'
                )
            ),
            'mono_depth.da3_baseline_ratio_log_sigma': LaunchConfiguration(
                'mono_depth.da3_baseline_ratio_log_sigma'
            ),
            'mono_depth.landmark_scale_flatness_radius': LaunchConfiguration(
                'mono_depth.landmark_scale_flatness_radius'
            ),
            'mono_depth.landmark_scale_max_relative_depth_variation': (
                LaunchConfiguration(
                    'mono_depth.landmark_scale_max_relative_depth_variation'
                )
            ),
            'mono_depth.depth_weight_range_ref': '20.0',
        }.items(),
    )

    dense_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dense_mapping'),
                'launch',
                'dense_mapping.launch.py',
            ])
        ),
        launch_arguments={
            'robot': 'a5',
            'frame_id.map': 'a5/map',
            'frame_id.odometry': 'world',
            'point_stride': '4',
            'max_points_per_view': '100000',
            'max_points_per_submap': '200000',
            'max_runs_per_submap': '5',
            'min_depth_m': '0.1',
            'max_depth_m': '100.0',
            'submap.metric_scale_method': LaunchConfiguration(
                'dense_mapping.submap_metric_scale_method'
            ),
            'submap.anchor_method': LaunchConfiguration(
                'dense_mapping.submap_anchor_method'
            ),
            'geometry_filter.enabled': LaunchConfiguration(
                'dense_mapping.geometry_filter_enabled'
            ),
            'geometry_filter.pose_source': LaunchConfiguration(
                'dense_mapping.geometry_filter_pose_source'
            ),
            'geometry_filter.max_relative_depth_error': LaunchConfiguration(
                'dense_mapping.geometry_filter_max_relative_depth_error'
            ),
            'geometry_filter.visualization_max_relative_error': (
                LaunchConfiguration(
                    'dense_mapping.geometry_filter_visualization_max_relative_error'
                )
            ),
            'rerun.enabled': 'true',
            'rerun.application_id': LaunchConfiguration(
                'rerun_application_id'
            ),
            'rerun.recording_id': LaunchConfiguration('rerun_recording_id'),
            'rerun.host': LaunchConfiguration('rerun_host'),
            'rerun.entity_prefix': 'a5/dense_mapping',
            'rerun.point_radius': '1.0',
            'sparse_global_ba.enabled': 'false',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', '/tmp/ros_logs'),
        rosbag_path_arg,
        rosbag_play_duration_arg,
        rosbag_rate_arg,
        start_zenoh_router_arg,
        rerun_host_arg,
        rerun_application_id_arg,
        rerun_recording_id_arg,
        geometry_filter_enabled_arg,
        submap_metric_scale_method_arg,
        submap_anchor_method_arg,
        geometry_filter_pose_source_arg,
        geometry_filter_max_error_arg,
        geometry_filter_visualization_max_error_arg,
        mono_depth_engine_arg,
        mono_depth_da3_keyframe_selection_method_arg,
        mono_depth_da3_keyframe_skip_arg,
        mono_depth_da3_keyframe_covisibility_threshold_arg,
        mono_depth_min_distance_arg,
        mono_depth_min_confidence_arg,
        mono_depth_scale_alignment_method_arg,
        mono_depth_da3_essential_factors_enabled_arg,
        mono_depth_da3_baseline_ratio_factors_enabled_arg,
        mono_depth_da3_baseline_ratio_log_sigma_arg,
        mono_depth_landmark_scale_flatness_radius_arg,
        mono_depth_landmark_scale_max_relative_depth_variation_arg,
        dense_mapping_launch,
        TimerAction(period=1.0, actions=[mono_launch]),
    ])
