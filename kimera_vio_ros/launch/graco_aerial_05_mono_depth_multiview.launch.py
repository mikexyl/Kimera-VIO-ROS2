from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


DA3_TWO_VIEW_ENGINE = (
    '/home/mikexyl/workspaces/xfeat_cpp_ws/xfeat-cpp/onnx_model/'
    'mono_depth/depth_anything_v3/'
    'DA3-LARGE-1.1_multiview_v2_350x504_fp16.engine'
)


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
    mono_depth_engine_arg = DeclareLaunchArgument(
        'mono_depth.engine_path',
        default_value=DA3_TWO_VIEW_ENGINE,
        description='Pose-free two-view DA3 TensorRT engine.',
    )
    mono_depth_da3_keyframe_selection_method_arg = DeclareLaunchArgument(
        'mono_depth.da3_keyframe_selection_method',
        default_value='distance',
        description='DA3 two-view endpoint selection: distance or fixed_skip.',
    )
    mono_depth_da3_keyframe_skip_arg = DeclareLaunchArgument(
        'mono_depth.da3_keyframe_skip',
        default_value='0',
        description='VIO keyframes held between DA3 endpoints in fixed_skip mode.',
    )
    mono_depth_min_distance_arg = DeclareLaunchArgument(
        'mono_depth.min_keyframe_distance_m',
        default_value='1.0',
        description='Minimum odometry camera-center displacement for DA3.',
    )
    mono_depth_min_confidence_arg = DeclareLaunchArgument(
        'mono_depth.min_confidence',
        default_value='1.1',
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
        default_value='true',
        description=(
            'Add DA3 essential-matrix factors to the fixed-lag smoother.'
        ),
    )
    mono_depth_da3_baseline_ratio_factors_enabled_arg = DeclareLaunchArgument(
        'mono_depth.da3_baseline_ratio_factors_enabled',
        default_value='true',
        description=(
            'Add scale-free consecutive DA3 baseline-ratio factors to the smoother.'
        ),
    )
    mono_depth_da3_baseline_ratio_log_sigma_arg = DeclareLaunchArgument(
        'mono_depth.da3_baseline_ratio_log_sigma',
        default_value='0.25',
        description='Standard deviation of the DA3 log baseline-ratio residual.',
    )
    mono_depth_icp_only_da3_overlap_fusion_arg = DeclareLaunchArgument(
        'mono_depth.icp_only_da3_overlap_fusion',
        default_value='true',
        description=(
            'Use DA3 shared-view scale chaining instead of ICP in the isolated path.'
        ),
    )
    mono_depth_visualize_landmark_scale_alignment_arg = DeclareLaunchArgument(
        'mono_depth.visualize_landmark_scale_alignment',
        default_value='true',
        description='Log landmark scale weights and rejection reasons to Rerun.',
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
            'topic.image': '/camera_left/image_raw',
            'topic.imu.data': '/gnss/imu',
            'topic.camera.info': '/camera_left/camera_info',
            'use_camera_info': 'false',
            'use_sim_time': 'true',
            'start_zenoh_router': LaunchConfiguration('start_zenoh_router'),
            'rerun_host': LaunchConfiguration('rerun_host'),
            'rerun_recording_id': 'graco-aerial-05-da3-two-view',
            'rosbag_play': 'true',
            'rosbag_path': LaunchConfiguration('rosbag_path'),
            'rosbag_play_delay': '4.0',
            'rosbag_rate': LaunchConfiguration('rosbag_rate'),
            'rosbag_play_duration': LaunchConfiguration(
                'rosbag_play_duration'
            ),
            'mono_depth.enabled': 'true',
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
            'mono_depth.icp_only_da3_overlap_fusion': LaunchConfiguration(
                'mono_depth.icp_only_da3_overlap_fusion'
            ),
            'mono_depth.visualize_landmark_scale_alignment': (
                LaunchConfiguration(
                    'mono_depth.visualize_landmark_scale_alignment'
                )
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
            'mono_depth.visualize_weights': 'false',
            'dense_map.enabled': 'true',
            'dense_map.voxel_resolution': '0.5',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', '/tmp/ros_logs'),
        rosbag_path_arg,
        rosbag_play_duration_arg,
        rosbag_rate_arg,
        start_zenoh_router_arg,
        rerun_host_arg,
        mono_depth_engine_arg,
        mono_depth_da3_keyframe_selection_method_arg,
        mono_depth_da3_keyframe_skip_arg,
        mono_depth_min_distance_arg,
        mono_depth_min_confidence_arg,
        mono_depth_scale_alignment_method_arg,
        mono_depth_da3_essential_factors_enabled_arg,
        mono_depth_da3_baseline_ratio_factors_enabled_arg,
        mono_depth_da3_baseline_ratio_log_sigma_arg,
        mono_depth_icp_only_da3_overlap_fusion_arg,
        mono_depth_visualize_landmark_scale_alignment_arg,
        mono_depth_landmark_scale_flatness_radius_arg,
        mono_depth_landmark_scale_max_relative_depth_variation_arg,
        mono_launch,
    ])
