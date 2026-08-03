import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _start_vio(context, node):
    delay = float(LaunchConfiguration('vio_start_delay').perform(context))
    if delay < 0.0:
        raise RuntimeError('vio_start_delay must be non-negative')
    if delay == 0.0:
        return [node]
    return [TimerAction(period=delay, actions=[node])]


def _rosbag_actions(context):
    if not _as_bool(LaunchConfiguration('rosbag_play').perform(context)):
        return []

    bag_path = LaunchConfiguration('rosbag_path').perform(context)
    if not bag_path or not Path(bag_path).is_dir():
        raise RuntimeError('rosbag_path must name an existing bag directory')

    delay = float(LaunchConfiguration('rosbag_play_delay').perform(context))
    duration = float(
        LaunchConfiguration('rosbag_play_duration').perform(context)
    )
    if delay < 0.0 or duration < 0.0:
        raise RuntimeError(
            'rosbag_play_delay and rosbag_play_duration must be non-negative'
        )

    source_imu = LaunchConfiguration(
        'rosbag_source_imu_topic'
    ).perform(context)
    source_image = LaunchConfiguration(
        'rosbag_source_image_topic'
    ).perform(context)
    target_imu = LaunchConfiguration('topic.imu.data').perform(context)
    target_image = LaunchConfiguration('topic.image').perform(context)
    rate = LaunchConfiguration('rosbag_rate').perform(context)
    publish_clock = _as_bool(
        LaunchConfiguration('rosbag_publish_clock').perform(context)
    )

    command = ['ros2', 'bag', 'play', bag_path]
    if publish_clock:
        command.append('--clock')
    command.extend(
        [
            '-r',
            rate,
            '--topics',
            source_imu,
            source_image,
            '--remap',
            f'{source_imu}:={target_imu}',
            f'{source_image}:={target_image}',
        ]
    )
    player = ExecuteProcess(cmd=command, output='screen')
    actions = [
        player if delay == 0.0 else TimerAction(
            period=delay,
            actions=[player],
        )
    ]
    if duration > 0.0:
        actions.append(
            TimerAction(
                period=delay + duration,
                actions=[
                    EmitEvent(
                        event=Shutdown(
                            reason='rosbag_play_duration elapsed'
                        )
                    )
                ],
            )
        )
    return actions


def generate_launch_description():
    workspace_root = Path(os.environ.get('SB_SLAM_ROS2_WS', os.getcwd()))
    default_mono_depth_engine = str(
        workspace_root /
        'src' /
        'xfeat-cpp' /
        'onnx_model' /
        'DA3METRIC-LARGE_280x504_fp16.engine'
    )

    dataset_arg = DeclareLaunchArgument(
        'dataset_name',
        default_value='EurocMono',
        description='Name of the dataset whose parameters should be used.'
    )
    parallel_arg = DeclareLaunchArgument(
        'parallel',
        default_value='false',
        description='Unused placeholder kept for parity with the ROS1 launch file.'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true.'
    )
    start_zenoh_router_arg = DeclareLaunchArgument(
        'start_zenoh_router',
        default_value='true',
        description='Start the rmw_zenoh router from this launch file.'
    )
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='Unique integer id of this robot.'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='na',
        description='Robot name, usually supplied by an enclosing namespace.'
    )
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Optional parent namespace for the kimera_vio node.'
    )
    log_output_arg = DeclareLaunchArgument(
        'log_output',
        default_value='false',
        description='Enable logging to the output path.'
    )
    log_output_path_arg = DeclareLaunchArgument(
        'log_output_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('kimera_vio_ros'),
            'output_logs'
        ]),
        description='Directory where module logs should be stored.'
    )
    use_lcd_arg = DeclareLaunchArgument(
        'use_lcd',
        default_value='0',
        description='Enable the loop-closure detector if true.'
    )
    bow_batch_size_arg = DeclareLaunchArgument(
        'bow_batch_size',
        default_value='5',
        description='Number of BoW vectors published in each query message.'
    )
    bow_skip_num_arg = DeclareLaunchArgument(
        'bow_skip_num',
        default_value='1',
        description='Publish every Nth BoW vector.'
    )
    publish_vlc_frames_arg = DeclareLaunchArgument(
        'publish_vlc_frames',
        default_value='true',
        description='Publish local VLC frames for distributed loop closure.'
    )
    bridge_enabled_arg = DeclareLaunchArgument(
        'multi_robot_bridge.enabled', default_value='false',
        description='Enable the Kimera multi-robot transport bridge.'
    )
    descriptor_batch_size_arg = DeclareLaunchArgument(
        'multi_robot_bridge.descriptor_batch_size', default_value='5'
    )
    descriptor_stride_arg = DeclareLaunchArgument(
        'multi_robot_bridge.descriptor_stride', default_value='1'
    )
    verification_batch_size_arg = DeclareLaunchArgument(
        'multi_robot_bridge.verification_frame_batch_size', default_value='50'
    )
    bridge_publish_frames_arg = DeclareLaunchArgument(
        'multi_robot_bridge.publish_verification_frames', default_value='true'
    )
    bridge_flush_period_arg = DeclareLaunchArgument(
        'multi_robot_bridge.flush_period_s', default_value='1.0'
    )
    model_xfeat_arg = DeclareLaunchArgument('models.xfeat', default_value='')
    model_xfeat_bilinear_arg = DeclareLaunchArgument(
        'models.xfeat_interp_bilinear', default_value=''
    )
    model_xfeat_bicubic_arg = DeclareLaunchArgument(
        'models.xfeat_interp_bicubic', default_value=''
    )
    model_xfeat_nearest_arg = DeclareLaunchArgument(
        'models.xfeat_interp_nearest', default_value=''
    )
    model_lightglue_frontend_arg = DeclareLaunchArgument(
        'models.lightglue_frontend', default_value=''
    )
    model_lightglue_lcd_arg = DeclareLaunchArgument(
        'models.lightglue_lcd', default_value=''
    )
    model_jist_arg = DeclareLaunchArgument('models.jist', default_value='')
    model_mixvpr_arg = DeclareLaunchArgument(
        'models.mixvpr', default_value=''
    )
    jist_frame_refinement_arg = DeclareLaunchArgument(
        'jist_frame_refinement', default_value='false'
    )
    params_folder_arg = DeclareLaunchArgument(
        'params_folder',
        default_value=PathJoinSubstitution([
            FindPackageShare('kimera_vio_ros'),
            'param',
            LaunchConfiguration('dataset_name')
        ]),
        description='Directory that contains Kimera-VIO parameter files.'
    )
    topic_image_arg = DeclareLaunchArgument(
        'topic.image',
        default_value='/cam0/image_raw',
        description='Camera image topic.'
    )
    topic_imu_data_arg = DeclareLaunchArgument(
        'topic.imu.data',
        default_value='/imu0',
        description='IMU topic.'
    )
    use_external_odom_arg = DeclareLaunchArgument(
        'use_external_odom',
        default_value='false',
        description='Fuse an external odometry input when true.'
    )
    topic_external_odom_arg = DeclareLaunchArgument(
        'topic.external_odom',
        default_value='external_odom',
        description='External odometry input topic.'
    )
    use_camera_info_arg = DeclareLaunchArgument(
        'use_camera_info',
        default_value='false',
        description='Subscribe to camera info topics if true.'
    )
    topic_camera_info_arg = DeclareLaunchArgument(
        'topic.camera.info',
        default_value='/cam0/camera_info',
        description='Camera info topic.'
    )
    frame_id_base_link_arg = DeclareLaunchArgument(
        'frame_id.base_link',
        default_value='base_link',
        description='Base link frame id.'
    )
    frame_id_odom_arg = DeclareLaunchArgument(
        'frame_id.odom',
        default_value='odom',
        description='Odometry frame id.'
    )
    frame_id_map_arg = DeclareLaunchArgument(
        'frame_id.map',
        default_value='map',
        description='Map frame id.'
    )
    frame_id_world_arg = DeclareLaunchArgument(
        'frame_id.world',
        default_value='world',
        description='World frame id.'
    )
    verbosity_arg = DeclareLaunchArgument(
        'verbosity',
        default_value='0',
        description='Glog verbosity level.'
    )
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='false',
        description='Enable OpenCV visualizations if true.'
    )
    use_rerun_visualizer_arg = DeclareLaunchArgument(
        'use_rerun_visualizer',
        default_value='true',
        description='Enable the Rerun visualizer independently of OpenCV visualizations.'
    )
    dense_mapping_publisher_enabled_arg = DeclareLaunchArgument(
        'dense_mapping.publisher_enabled',
        default_value='false',
        description='Publish sparse keyframes and valid canonical DA3 runs.'
    )
    rerun_host_arg = DeclareLaunchArgument(
        'rerun_host',
        default_value='rerun+http://127.0.0.1:9876/proxy',
        description='Rerun gRPC endpoint.'
    )
    rerun_recording_id_arg = DeclareLaunchArgument(
        'rerun_recording_id',
        default_value='',
        description='Optional Rerun recording id.'
    )
    rerun_application_id_arg = DeclareLaunchArgument(
        'rerun_application_id',
        default_value='kimera_vio',
        description='Rerun application id.'
    )
    rerun_result_dir_arg = DeclareLaunchArgument(
        'rerun_result_dir',
        default_value='',
        description='Optional directory for Rerun side outputs. Empty disables file output.'
    )
    rerun_visualization_profile_arg = DeclareLaunchArgument(
        'rerun_visualization_profile',
        default_value='full',
        description='VIO Rerun payload profile: full or minimal.'
    )
    rerun_tracking_image_jpeg_quality_arg = DeclareLaunchArgument(
        'rerun_tracking_image_jpeg_quality',
        default_value='80',
        description='JPEG quality for the minimal Rerun profile.'
    )
    mono_depth_enabled_arg = DeclareLaunchArgument(
        'mono_depth.enabled',
        default_value='false',
        description='Run DA3 monocular depth on mono keyframes.'
    )
    mono_depth_engine_path_arg = DeclareLaunchArgument(
        'mono_depth.engine_path',
        default_value=default_mono_depth_engine,
        description='TensorRT engine path for DA3 monocular depth.'
    )
    mono_depth_mode_arg = DeclareLaunchArgument(
        'mono_depth.mode',
        default_value='single_view',
        description='DA3 monocular depth mode: single_view or multi_view.'
    )
    mono_depth_da3_keyframe_selection_method_arg = DeclareLaunchArgument(
        'mono_depth.da3_keyframe_selection_method',
        default_value='distance',
        description='DA3 two-view endpoint selection: distance, fixed_skip, or covisibility.'
    )
    mono_depth_da3_keyframe_skip_arg = DeclareLaunchArgument(
        'mono_depth.da3_keyframe_skip',
        default_value='0',
        description='VIO keyframes held between DA3 endpoints in fixed_skip mode.'
    )
    mono_depth_da3_keyframe_covisibility_threshold_arg = DeclareLaunchArgument(
        'mono_depth.da3_keyframe_covisibility_threshold',
        default_value='0.5',
        description='Run two-view DA3 when reference-track covisibility falls below this fraction.'
    )
    mono_depth_min_keyframe_distance_arg = DeclareLaunchArgument(
        'mono_depth.min_keyframe_distance_m',
        default_value='3.0',
        description='Minimum odometry camera-center displacement in meters before running two-view DA3.'
    )
    mono_depth_point_stride_arg = DeclareLaunchArgument(
        'mono_depth.point_stride',
        default_value='4',
        description='Pixel sampling stride for DA3 point cloud back-projection.'
    )
    mono_depth_max_points_per_keyframe_arg = DeclareLaunchArgument(
        'mono_depth.max_points_per_keyframe',
        default_value='1000000',
        description='Maximum DA3 point cloud samples added per keyframe.'
    )
    mono_depth_min_depth_arg = DeclareLaunchArgument(
        'mono_depth.min_depth_m',
        default_value='0.1',
        description='Minimum valid DA3 depth in meters.'
    )
    mono_depth_max_depth_arg = DeclareLaunchArgument(
        'mono_depth.max_depth_m',
        default_value='100.0',
        description='Maximum valid DA3 depth in meters.'
    )
    mono_depth_depth_weighting_enabled_arg = DeclareLaunchArgument(
        'mono_depth.depth_weighting_enabled',
        default_value='true',
        description='Enable depth-normal and range weighting for mono-depth ICP.'
    )
    mono_depth_depth_weight_normal_radius_arg = DeclareLaunchArgument(
        'mono_depth.depth_weight_normal_radius',
        default_value='2',
        description='Pixel radius for central-difference mono-depth surface normals.'
    )
    mono_depth_depth_weight_min_arg = DeclareLaunchArgument(
        'mono_depth.depth_weight_min',
        default_value='0.05',
        description='Minimum grazing-angle mono-depth ICP weight.'
    )
    mono_depth_depth_weight_grazing_power_arg = DeclareLaunchArgument(
        'mono_depth.depth_weight_grazing_power',
        default_value='1.0',
        description='Power applied to mono-depth viewing-angle confidence.'
    )
    mono_depth_depth_weight_range_ref_arg = DeclareLaunchArgument(
        'mono_depth.depth_weight_range_ref',
        default_value='0.0',
        description='Reference range for optional mono-depth range weighting; 0 disables it.'
    )
    mono_depth_depth_weight_range_power_arg = DeclareLaunchArgument(
        'mono_depth.depth_weight_range_power',
        default_value='2.0',
        description='Range falloff power for mono-depth ICP weights.'
    )
    mono_depth_depth_weight_range_min_arg = DeclareLaunchArgument(
        'mono_depth.depth_weight_range_min',
        default_value='0.05',
        description='Minimum range multiplier for mono-depth ICP weights.'
    )
    mono_depth_min_confidence_arg = DeclareLaunchArgument(
        'mono_depth.min_confidence',
        default_value='1.1',
        description='Minimum DA3 confidence in multi_view mode; 0 disables confidence filtering.'
    )
    mono_depth_visualize_confidence_arg = DeclareLaunchArgument(
        'mono_depth.visualize_confidence',
        default_value='false',
        description='Log the newest multi-view DA3 confidence heatmap and threshold mask to Rerun.'
    )
    mono_depth_verbose_arg = DeclareLaunchArgument(
        'mono_depth.verbose',
        default_value='false',
        description='Enable verbose DA3 TensorRT logging.'
    )
    mono_depth_scale_alignment_method_arg = DeclareLaunchArgument(
        'mono_depth.scale_alignment_method',
        default_value='none',
        description='Mono-depth scale alignment: none, relative_pose, or landmarks.'
    )
    mono_depth_da3_essential_factors_enabled_arg = DeclareLaunchArgument(
        'mono_depth.da3_essential_factors_enabled',
        default_value='false',
        description=(
            'Add DA3 rotation and directed translation constraints to the smoother.'
        ),
    )
    mono_depth_da3_baseline_ratio_factors_enabled_arg = DeclareLaunchArgument(
        'mono_depth.da3_baseline_ratio_factors_enabled',
        default_value='false',
        description=(
            'Add scale-free consecutive DA3 baseline-ratio constraints to the smoother.'
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
        description='Pixel radius used to detect depth edges around landmark-scale samples.'
    )
    mono_depth_landmark_scale_max_relative_depth_variation_arg = DeclareLaunchArgument(
        'mono_depth.landmark_scale_max_relative_depth_variation',
        default_value='0.15',
        description='Maximum local relative depth variation before rejecting a landmark-scale sample.'
    )
    rosbag_play_arg = DeclareLaunchArgument(
        'rosbag_play',
        default_value='false',
        description='Start ros2 bag playback from this launch file.'
    )
    rosbag_publish_clock_arg = DeclareLaunchArgument(
        'rosbag_publish_clock',
        default_value='true',
        description='Publish /clock from this bag player.'
    )
    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value='',
        description='ROS2 bag directory to play when rosbag_play is true.'
    )
    rosbag_source_image_topic_arg = DeclareLaunchArgument(
        'rosbag_source_image_topic',
        default_value='/cam0/image_raw',
        description='Image topic stored in the input bag.'
    )
    rosbag_source_imu_topic_arg = DeclareLaunchArgument(
        'rosbag_source_imu_topic',
        default_value='/imu0',
        description='IMU topic stored in the input bag.'
    )
    rosbag_play_delay_arg = DeclareLaunchArgument(
        'rosbag_play_delay',
        default_value='5.0',
        description='Seconds to wait before starting ros2 bag playback.'
    )
    rosbag_rate_arg = DeclareLaunchArgument(
        'rosbag_rate',
        default_value='1.0',
        description='ros2 bag play rate.'
    )
    rosbag_play_duration_arg = DeclareLaunchArgument(
        'rosbag_play_duration',
        default_value='0.0',
        description='Seconds of rosbag playback before shutting down the launch. 0 disables the timed shutdown.'
    )
    vio_start_delay_arg = DeclareLaunchArgument(
        'vio_start_delay',
        default_value='1.0',
        description='Seconds to wait before starting the VIO node.'
    )

    node_arguments = [
        ['--use_lcd=', LaunchConfiguration('use_lcd')],
        ['--use_external_odometry=', LaunchConfiguration('use_external_odom')],
        ['--flagfile=', PathJoinSubstitution([LaunchConfiguration('params_folder'), 'flags', 'Mesher.flags'])],
        ['--flagfile=', PathJoinSubstitution([LaunchConfiguration('params_folder'), 'flags', 'VioBackend.flags'])],
        ['--flagfile=', PathJoinSubstitution([LaunchConfiguration('params_folder'), 'flags', 'RegularVioBackend.flags'])],
        ['--flagfile=', PathJoinSubstitution([LaunchConfiguration('params_folder'), 'flags', 'Visualizer3D.flags'])],
        '--logtostderr=1',
        '--colorlogtostderr=1',
        '--log_prefix=1',
        ['--v=', LaunchConfiguration('verbosity')],
        ['--log_output=', LaunchConfiguration('log_output')],
        ['--output_path=', LaunchConfiguration('log_output_path')],
        ['--visualize=', LaunchConfiguration('visualize')],
    ]

    node_parameters = [{
        'params_folder': LaunchConfiguration('params_folder'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'use_lcd': LaunchConfiguration('use_lcd'),
        'use_external_odom': LaunchConfiguration('use_external_odom'),
        'robot_id': LaunchConfiguration('robot_id'),
        'robot_name': LaunchConfiguration('robot_name'),
        'bow_batch_size': LaunchConfiguration('bow_batch_size'),
        'bow_skip_num': LaunchConfiguration('bow_skip_num'),
        'publish_vlc_frames': LaunchConfiguration('publish_vlc_frames'),
        'multi_robot_bridge.enabled': LaunchConfiguration('multi_robot_bridge.enabled'),
        'multi_robot_bridge.descriptor_batch_size': LaunchConfiguration('multi_robot_bridge.descriptor_batch_size'),
        'multi_robot_bridge.descriptor_stride': LaunchConfiguration('multi_robot_bridge.descriptor_stride'),
        'multi_robot_bridge.verification_frame_batch_size': LaunchConfiguration('multi_robot_bridge.verification_frame_batch_size'),
        'multi_robot_bridge.publish_verification_frames': LaunchConfiguration('multi_robot_bridge.publish_verification_frames'),
        'multi_robot_bridge.flush_period_s': LaunchConfiguration('multi_robot_bridge.flush_period_s'),
        'models.xfeat': LaunchConfiguration('models.xfeat'),
        'models.xfeat_interp_bilinear': LaunchConfiguration('models.xfeat_interp_bilinear'),
        'models.xfeat_interp_bicubic': LaunchConfiguration('models.xfeat_interp_bicubic'),
        'models.xfeat_interp_nearest': LaunchConfiguration('models.xfeat_interp_nearest'),
        'models.lightglue_frontend': LaunchConfiguration('models.lightglue_frontend'),
        'models.lightglue_lcd': LaunchConfiguration('models.lightglue_lcd'),
        'models.jist': LaunchConfiguration('models.jist'),
        'models.mixvpr': LaunchConfiguration('models.mixvpr'),
        'jist_frame_refinement': LaunchConfiguration('jist_frame_refinement'),
        'frame_id.base_link': LaunchConfiguration('frame_id.base_link'),
        'frame_id.odom': LaunchConfiguration('frame_id.odom'),
        'frame_id.map': LaunchConfiguration('frame_id.map'),
        'frame_id.world': LaunchConfiguration('frame_id.world'),
        'use_camera_info': LaunchConfiguration('use_camera_info'),
        'velocity_det_threshold': 0.1,
        'position_det_threshold': 0.3,
        'mono_ransac_threshold': 30,
        'use_rerun_visualizer': LaunchConfiguration('use_rerun_visualizer'),
        'dense_mapping.publisher_enabled': LaunchConfiguration(
            'dense_mapping.publisher_enabled'
        ),
        'rerun_host': LaunchConfiguration('rerun_host'),
        'rerun_application_id': LaunchConfiguration('rerun_application_id'),
        'rerun_recording_id': LaunchConfiguration('rerun_recording_id'),
        'rerun_result_dir': LaunchConfiguration('rerun_result_dir'),
        'rerun_visualization_profile': LaunchConfiguration(
            'rerun_visualization_profile'
        ),
        'rerun_tracking_image_jpeg_quality': LaunchConfiguration(
            'rerun_tracking_image_jpeg_quality'
        ),
        'mono_depth.enabled': LaunchConfiguration('mono_depth.enabled'),
        'mono_depth.engine_path': LaunchConfiguration('mono_depth.engine_path'),
        'mono_depth.mode': LaunchConfiguration('mono_depth.mode'),
        'mono_depth.da3_keyframe_selection_method': LaunchConfiguration(
            'mono_depth.da3_keyframe_selection_method'
        ),
        'mono_depth.da3_keyframe_skip': LaunchConfiguration(
            'mono_depth.da3_keyframe_skip'
        ),
        'mono_depth.da3_keyframe_covisibility_threshold': LaunchConfiguration(
            'mono_depth.da3_keyframe_covisibility_threshold'
        ),
        'mono_depth.min_keyframe_distance_m': LaunchConfiguration('mono_depth.min_keyframe_distance_m'),
        'mono_depth.point_stride': LaunchConfiguration('mono_depth.point_stride'),
        'mono_depth.max_points_per_keyframe': LaunchConfiguration('mono_depth.max_points_per_keyframe'),
        'mono_depth.min_depth_m': LaunchConfiguration('mono_depth.min_depth_m'),
        'mono_depth.max_depth_m': LaunchConfiguration('mono_depth.max_depth_m'),
        'mono_depth.depth_weighting_enabled': LaunchConfiguration('mono_depth.depth_weighting_enabled'),
        'mono_depth.depth_weight_normal_radius': LaunchConfiguration('mono_depth.depth_weight_normal_radius'),
        'mono_depth.depth_weight_min': LaunchConfiguration('mono_depth.depth_weight_min'),
        'mono_depth.depth_weight_grazing_power': LaunchConfiguration('mono_depth.depth_weight_grazing_power'),
        'mono_depth.depth_weight_range_ref': LaunchConfiguration('mono_depth.depth_weight_range_ref'),
        'mono_depth.depth_weight_range_power': LaunchConfiguration('mono_depth.depth_weight_range_power'),
        'mono_depth.depth_weight_range_min': LaunchConfiguration('mono_depth.depth_weight_range_min'),
        'mono_depth.min_confidence': LaunchConfiguration('mono_depth.min_confidence'),
        'mono_depth.visualize_confidence': LaunchConfiguration('mono_depth.visualize_confidence'),
        'mono_depth.verbose': LaunchConfiguration('mono_depth.verbose'),
        'mono_depth.scale_alignment_method': LaunchConfiguration('mono_depth.scale_alignment_method'),
        'mono_depth.da3_essential_factors_enabled': LaunchConfiguration('mono_depth.da3_essential_factors_enabled'),
        'mono_depth.da3_baseline_ratio_factors_enabled': LaunchConfiguration('mono_depth.da3_baseline_ratio_factors_enabled'),
        'mono_depth.da3_baseline_ratio_log_sigma': LaunchConfiguration('mono_depth.da3_baseline_ratio_log_sigma'),
        'mono_depth.landmark_scale_flatness_radius': LaunchConfiguration('mono_depth.landmark_scale_flatness_radius'),
        'mono_depth.landmark_scale_max_relative_depth_variation': LaunchConfiguration('mono_depth.landmark_scale_max_relative_depth_variation'),
    }]

    remappings = [
        ('image', LaunchConfiguration('topic.image')),
        ('imu/data', LaunchConfiguration('topic.imu.data')),
        ('external_odom', LaunchConfiguration('topic.external_odom')),
        ('camera_info', LaunchConfiguration('topic.camera.info')),
        ('odometry', 'odometry'),
        ('resiliency', 'resiliency'),
        ('imu_bias', 'imu_bias'),
        ('optimized_trajectory', 'optimized_trajectory'),
        ('pose_graph', 'pose_graph'),
        ('optimized_odometry', 'optimized_odometry'),
        ('mesh', 'mesh'),
        ('frontend_stats', 'frontend_stats'),
        ('debug_mesh_img', 'debug_mesh_img'),
        ('time_horizon_pointcloud', 'time_horizon_pointcloud'),
    ]

    kimera_vio_node = Node(
        package='kimera_vio_ros',
        executable='mono_vio_node',
        namespace=PathJoinSubstitution([
            LaunchConfiguration('robot_namespace'),
            'kimera_vio',
        ]),
        name='kimera_vio_ros_mono',
        output='screen',
        arguments=node_arguments,
        parameters=node_parameters,
        remappings=remappings,
        # prefix=['kitty -e gdb -ex run --args'],
    )

    zenoh_router = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('start_zenoh_router')),
        cmd=[
            'ros2',
            'run',
            'rmw_zenoh_cpp',
            'rmw_zenohd',
        ],
        output='screen',
    )

    start_kimera_vio_node = OpaqueFunction(
        function=_start_vio,
        args=[kimera_vio_node],
    )
    rosbag_actions = OpaqueFunction(function=_rosbag_actions)

    return LaunchDescription([
        dataset_arg,
        parallel_arg,
        use_sim_time_arg,
        start_zenoh_router_arg,
        robot_id_arg,
        robot_name_arg,
        robot_namespace_arg,
        log_output_arg,
        log_output_path_arg,
        use_lcd_arg,
        bow_batch_size_arg,
        bow_skip_num_arg,
        publish_vlc_frames_arg,
        bridge_enabled_arg,
        descriptor_batch_size_arg,
        descriptor_stride_arg,
        verification_batch_size_arg,
        bridge_publish_frames_arg,
        bridge_flush_period_arg,
        model_xfeat_arg,
        model_xfeat_bilinear_arg,
        model_xfeat_bicubic_arg,
        model_xfeat_nearest_arg,
        model_lightglue_frontend_arg,
        model_lightglue_lcd_arg,
        model_jist_arg,
        model_mixvpr_arg,
        jist_frame_refinement_arg,
        params_folder_arg,
        topic_image_arg,
        topic_imu_data_arg,
        use_external_odom_arg,
        topic_external_odom_arg,
        use_camera_info_arg,
        topic_camera_info_arg,
        frame_id_base_link_arg,
        frame_id_odom_arg,
        frame_id_map_arg,
        frame_id_world_arg,
        verbosity_arg,
        visualize_arg,
        use_rerun_visualizer_arg,
        dense_mapping_publisher_enabled_arg,
        rerun_host_arg,
        rerun_application_id_arg,
        rerun_recording_id_arg,
        rerun_result_dir_arg,
        rerun_visualization_profile_arg,
        rerun_tracking_image_jpeg_quality_arg,
        mono_depth_enabled_arg,
        mono_depth_engine_path_arg,
        mono_depth_mode_arg,
        mono_depth_da3_keyframe_selection_method_arg,
        mono_depth_da3_keyframe_skip_arg,
        mono_depth_da3_keyframe_covisibility_threshold_arg,
        mono_depth_min_keyframe_distance_arg,
        mono_depth_point_stride_arg,
        mono_depth_max_points_per_keyframe_arg,
        mono_depth_min_depth_arg,
        mono_depth_max_depth_arg,
        mono_depth_depth_weighting_enabled_arg,
        mono_depth_depth_weight_normal_radius_arg,
        mono_depth_depth_weight_min_arg,
        mono_depth_depth_weight_grazing_power_arg,
        mono_depth_depth_weight_range_ref_arg,
        mono_depth_depth_weight_range_power_arg,
        mono_depth_depth_weight_range_min_arg,
        mono_depth_min_confidence_arg,
        mono_depth_visualize_confidence_arg,
        mono_depth_verbose_arg,
        mono_depth_scale_alignment_method_arg,
        mono_depth_da3_essential_factors_enabled_arg,
        mono_depth_da3_baseline_ratio_factors_enabled_arg,
        mono_depth_da3_baseline_ratio_log_sigma_arg,
        mono_depth_landmark_scale_flatness_radius_arg,
        mono_depth_landmark_scale_max_relative_depth_variation_arg,
        rosbag_play_arg,
        rosbag_publish_clock_arg,
        rosbag_path_arg,
        rosbag_source_image_topic_arg,
        rosbag_source_imu_topic_arg,
        rosbag_play_delay_arg,
        rosbag_rate_arg,
        rosbag_play_duration_arg,
        vio_start_delay_arg,
        zenoh_router,
        start_kimera_vio_node,
        rosbag_actions,
    ])
