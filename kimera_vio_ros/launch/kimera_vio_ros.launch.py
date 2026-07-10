import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        default_value='Euroc',
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
    params_folder_arg = DeclareLaunchArgument(
        'params_folder',
        default_value=PathJoinSubstitution([
            FindPackageShare('kimera_vio_ros'),
            'param',
            LaunchConfiguration('dataset_name')
        ]),
        description='Directory that contains Kimera-VIO parameter files.'
    )
    topic_left_image_arg = DeclareLaunchArgument(
        'topic.left.image',
        default_value='/cam0/image_raw',
        description='Left camera image topic.'
    )
    topic_right_image_arg = DeclareLaunchArgument(
        'topic.right.image',
        default_value='/cam1/image_raw',
        description='Right camera image topic.'
    )
    topic_imu_data_arg = DeclareLaunchArgument(
        'topic.imu.data',
        default_value='/imu0',
        description='IMU topic.'
    )
    use_camera_info_arg = DeclareLaunchArgument(
        'use_camera_info',
        default_value='false',
        description='Subscribe to camera info topics if true.'
    )
    topic_left_info_arg = DeclareLaunchArgument(
        'topic.left.info',
        default_value='/cam0/camera_info',
        description='Left camera info topic.'
    )
    topic_right_info_arg = DeclareLaunchArgument(
        'topic.right.info',
        default_value='/cam1/camera_info',
        description='Right camera info topic.'
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
    rerun_result_dir_arg = DeclareLaunchArgument(
        'rerun_result_dir',
        default_value='',
        description='Optional directory for Rerun side outputs. Empty disables file output.'
    )
    mono_depth_enabled_arg = DeclareLaunchArgument(
        'mono_depth.enabled',
        default_value='false',
        description='Run DA3 monocular depth on keyframes and log an accumulated point cloud to Rerun.'
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
    mono_depth_keyframe_skip_arg = DeclareLaunchArgument(
        'mono_depth.keyframe_skip',
        default_value='0',
        description='Number of keyframes to skip between DA3 mono-depth inference runs.'
    )
    mono_depth_point_stride_arg = DeclareLaunchArgument(
        'mono_depth.point_stride',
        default_value='4',
        description='Pixel sampling stride for DA3 point cloud back-projection.'
    )
    mono_depth_max_points_per_keyframe_arg = DeclareLaunchArgument(
        'mono_depth.max_points_per_keyframe',
        default_value='5000',
        description='Maximum DA3 point cloud samples added per keyframe.'
    )
    mono_depth_visualization_point_stride_arg = DeclareLaunchArgument(
        'mono_depth.visualization_point_stride',
        default_value='4',
        description='Pixel sampling stride for mono-depth Rerun and dense-map visualization.'
    )
    mono_depth_visualization_max_points_per_keyframe_arg = DeclareLaunchArgument(
        'mono_depth.visualization_max_points_per_keyframe',
        default_value='5000',
        description='Maximum mono-depth samples per keyframe for Rerun and dense-map visualization.'
    )
    mono_depth_min_depth_arg = DeclareLaunchArgument(
        'mono_depth.min_depth_m',
        default_value='0.1',
        description='Minimum valid DA3 depth in meters.'
    )
    mono_depth_max_depth_arg = DeclareLaunchArgument(
        'mono_depth.max_depth_m',
        default_value='30.0',
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
    mono_depth_visualize_weights_arg = DeclareLaunchArgument(
        'mono_depth.visualize_weights',
        default_value='false',
        description='Publish a Rerun debug cloud colored by mono-depth ICP weights.'
    )
    mono_depth_point_radius_arg = DeclareLaunchArgument(
        'mono_depth.point_radius',
        default_value='0.005',
        description='Rerun point radius for the DA3 map.'
    )
    mono_depth_verbose_arg = DeclareLaunchArgument(
        'mono_depth.verbose',
        default_value='false',
        description='Enable verbose DA3 TensorRT logging.'
    )
    mono_depth_align_scale_with_landmarks_arg = DeclareLaunchArgument(
        'mono_depth.align_scale_with_landmarks',
        default_value='false',
        description='Align DA3 mono-depth scale from backend landmarks.'
    )
    dense_map_enabled_arg = DeclareLaunchArgument(
        'dense_map.enabled',
        default_value='true',
        description='Insert aligned mono depth keyframe clouds into the dense map backend.'
    )
    dense_map_backend_arg = DeclareLaunchArgument(
        'dense_map.backend',
        default_value='gaussian_voxel_map',
        description='Dense map backend implementation.'
    )
    dense_map_voxel_resolution_arg = DeclareLaunchArgument(
        'dense_map.voxel_resolution',
        default_value='0.15',
        description='Voxel resolution in meters for gaussian_voxel_map.'
    )
    dense_map_point_radius_arg = DeclareLaunchArgument(
        'dense_map.point_radius',
        default_value='0.025',
        description='Rerun point radius for dense map visualization.'
    )

    node_arguments = [
        ['--use_lcd=', LaunchConfiguration('use_lcd')],
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
        'robot_id': LaunchConfiguration('robot_id'),
        'robot_name': LaunchConfiguration('robot_name'),
        'bow_batch_size': LaunchConfiguration('bow_batch_size'),
        'bow_skip_num': LaunchConfiguration('bow_skip_num'),
        'publish_vlc_frames': LaunchConfiguration('publish_vlc_frames'),
        'frame_id.base_link': LaunchConfiguration('frame_id.base_link'),
        'frame_id.odom': LaunchConfiguration('frame_id.odom'),
        'frame_id.map': LaunchConfiguration('frame_id.map'),
        'frame_id.world': LaunchConfiguration('frame_id.world'),
        'use_camera_info': LaunchConfiguration('use_camera_info'),
        'velocity_det_threshold': 0.1,
        'position_det_threshold': 0.3,
        'stereo_ransac_threshold': 20,
        'mono_ransac_threshold': 30,
        'use_rerun_visualizer': LaunchConfiguration('use_rerun_visualizer'),
        'rerun_host': LaunchConfiguration('rerun_host'),
        'rerun_recording_id': LaunchConfiguration('rerun_recording_id'),
        'rerun_result_dir': LaunchConfiguration('rerun_result_dir'),
        'mono_depth.enabled': LaunchConfiguration('mono_depth.enabled'),
        'mono_depth.engine_path': LaunchConfiguration('mono_depth.engine_path'),
        'mono_depth.mode': LaunchConfiguration('mono_depth.mode'),
        'mono_depth.keyframe_skip': LaunchConfiguration('mono_depth.keyframe_skip'),
        'mono_depth.point_stride': LaunchConfiguration('mono_depth.point_stride'),
        'mono_depth.max_points_per_keyframe': LaunchConfiguration('mono_depth.max_points_per_keyframe'),
        'mono_depth.visualization_point_stride': LaunchConfiguration('mono_depth.visualization_point_stride'),
        'mono_depth.visualization_max_points_per_keyframe': LaunchConfiguration('mono_depth.visualization_max_points_per_keyframe'),
        'mono_depth.min_depth_m': LaunchConfiguration('mono_depth.min_depth_m'),
        'mono_depth.max_depth_m': LaunchConfiguration('mono_depth.max_depth_m'),
        'mono_depth.depth_weighting_enabled': LaunchConfiguration('mono_depth.depth_weighting_enabled'),
        'mono_depth.depth_weight_normal_radius': LaunchConfiguration('mono_depth.depth_weight_normal_radius'),
        'mono_depth.depth_weight_min': LaunchConfiguration('mono_depth.depth_weight_min'),
        'mono_depth.depth_weight_grazing_power': LaunchConfiguration('mono_depth.depth_weight_grazing_power'),
        'mono_depth.depth_weight_range_ref': LaunchConfiguration('mono_depth.depth_weight_range_ref'),
        'mono_depth.depth_weight_range_power': LaunchConfiguration('mono_depth.depth_weight_range_power'),
        'mono_depth.depth_weight_range_min': LaunchConfiguration('mono_depth.depth_weight_range_min'),
        'mono_depth.visualize_weights': LaunchConfiguration('mono_depth.visualize_weights'),
        'mono_depth.point_radius': LaunchConfiguration('mono_depth.point_radius'),
        'mono_depth.verbose': LaunchConfiguration('mono_depth.verbose'),
        'mono_depth.align_scale_with_landmarks': LaunchConfiguration('mono_depth.align_scale_with_landmarks'),
        'dense_map.enabled': LaunchConfiguration('dense_map.enabled'),
        'dense_map.backend': LaunchConfiguration('dense_map.backend'),
        'dense_map.voxel_resolution': LaunchConfiguration('dense_map.voxel_resolution'),
        'dense_map.point_radius': LaunchConfiguration('dense_map.point_radius'),
    }]

    remappings = [
        ('left/image', LaunchConfiguration('topic.left.image')),
        ('right/image', LaunchConfiguration('topic.right.image')),
        ('imu/data', LaunchConfiguration('topic.imu.data')),
        ('left/camera_info', LaunchConfiguration('topic.left.info')),
        ('right/camera_info', LaunchConfiguration('topic.right.info')),
        ('odometry', 'odometry'),
        ('resiliency', 'resiliency'),
        ('imu_bias', 'imu_bias'),
        ('optimized_trajectory', 'optimized_trajectory'),
        ('pose_graph', 'pose_graph'),
        ('pose_graph_incremental', 'pose_graph_incremental'),
        ('optimized_odometry', 'optimized_odometry'),
        ('bow_query', 'bow_query'),
        ('vlc_frames', 'vlc_frames'),
        ('vlc_frame_query', 'vlc_frame_query'),
        ('mesh', 'mesh'),
        ('frontend_stats', 'frontend_stats'),
        ('debug_mesh_img', 'debug_mesh_img'),
        ('time_horizon_pointcloud', 'time_horizon_pointcloud'),
    ]

    kimera_vio_node = Node(
        package='kimera_vio_ros',
        executable='stereo_vio_node',
        namespace='kimera_vio_ros',
        name='kimera_vio_ros',
        output='screen',
        arguments=node_arguments,
        parameters=node_parameters,
        remappings=remappings,
        # prefix=['kitty -e gdb -ex run --args'],
    )

    return LaunchDescription([
        dataset_arg,
        parallel_arg,
        use_sim_time_arg,
        robot_id_arg,
        robot_name_arg,
        log_output_arg,
        log_output_path_arg,
        use_lcd_arg,
        bow_batch_size_arg,
        bow_skip_num_arg,
        publish_vlc_frames_arg,
        params_folder_arg,
        topic_left_image_arg,
        topic_right_image_arg,
        topic_imu_data_arg,
        use_camera_info_arg,
        topic_left_info_arg,
        topic_right_info_arg,
        frame_id_base_link_arg,
        frame_id_odom_arg,
        frame_id_map_arg,
        frame_id_world_arg,
        verbosity_arg,
        visualize_arg,
        use_rerun_visualizer_arg,
        rerun_host_arg,
        rerun_recording_id_arg,
        rerun_result_dir_arg,
        mono_depth_enabled_arg,
        mono_depth_engine_path_arg,
        mono_depth_mode_arg,
        mono_depth_keyframe_skip_arg,
        mono_depth_point_stride_arg,
        mono_depth_max_points_per_keyframe_arg,
        mono_depth_visualization_point_stride_arg,
        mono_depth_visualization_max_points_per_keyframe_arg,
        mono_depth_min_depth_arg,
        mono_depth_max_depth_arg,
        mono_depth_depth_weighting_enabled_arg,
        mono_depth_depth_weight_normal_radius_arg,
        mono_depth_depth_weight_min_arg,
        mono_depth_depth_weight_grazing_power_arg,
        mono_depth_depth_weight_range_ref_arg,
        mono_depth_depth_weight_range_power_arg,
        mono_depth_depth_weight_range_min_arg,
        mono_depth_visualize_weights_arg,
        mono_depth_point_radius_arg,
        mono_depth_verbose_arg,
        mono_depth_align_scale_with_landmarks_arg,
        dense_map_enabled_arg,
        dense_map_backend_arg,
        dense_map_voxel_resolution_arg,
        dense_map_point_radius_arg,
        kimera_vio_node,
    ])
