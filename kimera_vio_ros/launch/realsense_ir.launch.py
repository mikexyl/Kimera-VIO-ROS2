from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    dataset_name_arg = DeclareLaunchArgument('dataset_name', default_value='RealSenseIR')
    parallel_arg = DeclareLaunchArgument('parallel', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    rectified_arg = DeclareLaunchArgument('rectified', default_value='false')
    use_lcd_arg = DeclareLaunchArgument('use_lcd', default_value='false')
    log_output_arg = DeclareLaunchArgument('log_output', default_value='false')

    # Include the main Kimera-VIO-ROS2 pipeline launch file (Python version)
    kimera_vio_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                'kimera_vio_ros.launch.py'
            )
        ),
        launch_arguments={
            'topic.left.image': '/camera/camera/infra1/image_rect_raw',
            'topic.right.image': '/camera/camera/infra2/image_rect_raw',
            'topic.left.info': '/camera/camera/infra1/camera_info',
            'topic.right.info': '/camera/camera/infra2/camera_info',
            'topic.imu.data': '/camera/camera/imu',
            'use_camera_info': 'true',
            # 'verbosity': '100',
        }.items()
    )

    # Include the RealSense2 camera launch file with IR cameras enabled
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                'realsense_only.launch.py'
            )
        )
    )

    # # Static transform publisher node
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_to_camera_link',
    #     arguments=['-0.006', '0.005', '0.012', '0.500', '-0.500', '0.500', '0.500', 'base_link', 'camera_link']
    # )

    # Group all actions
    group = GroupAction([
        dataset_name_arg,
        parallel_arg,
        use_sim_time_arg,
        rectified_arg,
        use_lcd_arg,
        log_output_arg,
        kimera_vio_ros_launch,
        realsense_launch
    ])

    return LaunchDescription([
        group,
        # static_tf_node
    ])
