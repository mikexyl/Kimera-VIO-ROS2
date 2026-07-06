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
        default_value='true',
        description='Enable OpenCV visualizations if true.'
    )
    use_rerun_visualizer_arg = DeclareLaunchArgument(
        'use_rerun_visualizer',
        default_value='false',
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
        'mono_ransac_threshold': 30,
        'use_rerun_visualizer': LaunchConfiguration('use_rerun_visualizer'),
        'rerun_host': LaunchConfiguration('rerun_host'),
        'rerun_recording_id': LaunchConfiguration('rerun_recording_id'),
        'rerun_result_dir': LaunchConfiguration('rerun_result_dir'),
    }]

    remappings = [
        ('image', LaunchConfiguration('topic.image')),
        ('imu/data', LaunchConfiguration('topic.imu.data')),
        ('camera_info', LaunchConfiguration('topic.camera.info')),
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
        executable='mono_vio_node',
        namespace='kimera_vio_ros',
        name='kimera_vio_ros_mono',
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
        topic_image_arg,
        topic_imu_data_arg,
        use_camera_info_arg,
        topic_camera_info_arg,
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
        kimera_vio_node,
    ])
