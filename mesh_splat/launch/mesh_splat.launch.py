from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mesh_topic = LaunchConfiguration("mesh_topic")
    texture_topic = LaunchConfiguration("texture_topic")
    entity_path = LaunchConfiguration("entity_path")
    rerun_url = LaunchConfiguration("rerun_url")
    application_id = LaunchConfiguration("application_id")
    wireframe_radius = LaunchConfiguration("wireframe_radius")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mesh_topic", default_value="mesh"),
            DeclareLaunchArgument("texture_topic", default_value="debug_mesh_img"),
            DeclareLaunchArgument("entity_path", default_value="map/mesh_splat/mesh"),
            DeclareLaunchArgument(
                "rerun_url", default_value="rerun+http://127.0.0.1:9876/proxy"
            ),
            DeclareLaunchArgument("application_id", default_value="mesh_splat"),
            DeclareLaunchArgument("wireframe_radius", default_value="0.002"),
            Node(
                package="mesh_splat",
                executable="mesh_splat_node",
                name="mesh_splat",
                output="screen",
                parameters=[
                    {
                        "mesh_topic": mesh_topic,
                        "texture_topic": texture_topic,
                        "entity_path": entity_path,
                        "rerun_url": rerun_url,
                        "application_id": application_id,
                        "wireframe_radius": wireframe_radius,
                    }
                ],
            ),
        ]
    )
