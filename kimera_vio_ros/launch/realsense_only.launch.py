from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
import os

def generate_launch_description():
    # Include the RealSense2 camera launch file with IR cameras enabled
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            # 'enable_infra': 'true',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'enable_sync': 'true',
            'pointcloud.enable': 'false',
            'unite_imu_method': '2',
        }.items()
    )

    return LaunchDescription([
        SetParameter(name='depth_module.emitter_enabled', value=0),
        realsense_launch
    ])
