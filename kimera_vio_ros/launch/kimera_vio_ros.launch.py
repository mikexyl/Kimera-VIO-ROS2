from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
    params_folder_arg = DeclareLaunchArgument(
        'params_folder',
        default_value=PathJoinSubstitution([
            FindPackageShare('kimera_vio_ros'),
            'param',
            LaunchConfiguration('dataset_name')
        ]),
        description='Directory that contains Kimera-VIO parameter files.'
    )
    path_to_vocab_arg = DeclareLaunchArgument(
        'path_to_vocab',
        default_value='/opt/overlay_ws/src/MIT-SPARK/Kimera-VIO/vocabulary/ORBvoc.yml',
        description='Absolute path to the ORB vocabulary file.'
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
        default_value='true',
        description='Enable OpenCV visualizations if true.'
    )

    node_arguments = [
        ['--use_lcd=', LaunchConfiguration('use_lcd')],
        '--vocabulary_path', LaunchConfiguration('path_to_vocab'),
        '--flagfile', PathJoinSubstitution([LaunchConfiguration('params_folder'), 'flags', 'Mesher.flags']),
        '--flagfile', PathJoinSubstitution([LaunchConfiguration('params_folder'), 'flags', 'VioBackend.flags']),
        '--flagfile', PathJoinSubstitution([LaunchConfiguration('params_folder'), 'flags', 'RegularVioBackend.flags']),
        '--flagfile', PathJoinSubstitution([LaunchConfiguration('params_folder'), 'flags', 'Visualizer3D.flags']),
        '--logtostderr', '1',
        '--colorlogtostderr', '1',
        '--log_prefix', '1',
        '--v', LaunchConfiguration('verbosity'),
        ['--log_output=', LaunchConfiguration('log_output')],
        '--output_path', LaunchConfiguration('log_output_path'),
        ['--visualize=', LaunchConfiguration('visualize')],
    ]

    node_parameters = [{
        'params_folder': LaunchConfiguration('params_folder'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'use_lcd': LaunchConfiguration('use_lcd'),
        'frame_id.base_link': LaunchConfiguration('frame_id.base_link'),
        'frame_id.map': LaunchConfiguration('frame_id.map'),
        'frame_id.world': LaunchConfiguration('frame_id.world'),
        'use_camera_info': LaunchConfiguration('use_camera_info'),
        'velocity_det_threshold': 0.1,
        'position_det_threshold': 0.3,
        'stereo_ransac_threshold': 20,
        'mono_ransac_threshold': 30,
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
    )

    return LaunchDescription([
        dataset_arg,
        parallel_arg,
        use_sim_time_arg,
        log_output_arg,
        log_output_path_arg,
        use_lcd_arg,
        params_folder_arg,
        path_to_vocab_arg,
        topic_left_image_arg,
        topic_right_image_arg,
        topic_imu_data_arg,
        use_camera_info_arg,
        topic_left_info_arg,
        topic_right_info_arg,
        frame_id_base_link_arg,
        frame_id_map_arg,
        frame_id_world_arg,
        verbosity_arg,
        visualize_arg,
        kimera_vio_node,
    ])
