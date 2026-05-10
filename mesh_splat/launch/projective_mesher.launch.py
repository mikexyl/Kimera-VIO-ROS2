from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time.",
    )
    mesher_input_topic_arg = DeclareLaunchArgument(
        "mesher_input_topic",
        default_value="/mesher_input",
        description="Generic projective_mesher_msgs/MesherInput topic.",
    )
    mesh_topic_arg = DeclareLaunchArgument(
        "mesh_topic",
        default_value="mesh",
        description="Output pcl_msgs/PolygonMesh topic.",
    )
    texture_topic_arg = DeclareLaunchArgument(
        "texture_topic",
        default_value="debug_mesh_img",
        description="Output texture image topic.",
    )
    mesh_frame_id_arg = DeclareLaunchArgument(
        "mesh_frame_id",
        default_value="",
        description="Override output mesh frame. Empty uses input header frame.",
    )
    max_triangle_side_arg = DeclareLaunchArgument(
        "max_triangle_side",
        default_value="2.0",
        description="Reject triangles with any side longer than this. <=0 disables.",
    )
    min_side_ratio_arg = DeclareLaunchArgument(
        "min_ratio_btw_largest_smallest_side",
        default_value="0.2",
        description="Reject elongated triangles with min_side/max_side below this.",
    )
    min_observations_arg = DeclareLaunchArgument(
        "min_observations",
        default_value="3",
        description="Minimum valid observations required to build a mesh.",
    )
    enable_rerun_arg = DeclareLaunchArgument(
        "enable_rerun",
        default_value="false",
        description="Log generated meshes to Rerun through aria_viz.",
    )
    rerun_host_arg = DeclareLaunchArgument(
        "rerun_host",
        default_value="rerun+http://127.0.0.1:9876/proxy",
        description="Rerun gRPC endpoint used by aria_viz.",
    )
    rerun_app_id_arg = DeclareLaunchArgument(
        "rerun_app_id",
        default_value="projective_mesher",
        description="Rerun application id.",
    )
    rerun_recording_id_arg = DeclareLaunchArgument(
        "rerun_recording_id",
        default_value="",
        description="Optional Rerun recording id. Empty lets aria_viz choose.",
    )
    rerun_entity_path_arg = DeclareLaunchArgument(
        "rerun_entity_path",
        default_value="map/projective_mesher/mesh",
        description="Rerun entity path for the generated mesh.",
    )
    rerun_camera_entity_path_arg = DeclareLaunchArgument(
        "rerun_camera_entity_path",
        default_value="map/projective_mesher/camera",
        description="Rerun entity path for the input camera pose.",
    )
    log_rerun_camera_pose_arg = DeclareLaunchArgument(
        "log_rerun_camera_pose",
        default_value="true",
        description="Log the input camera pose to Rerun.",
    )

    mesher_node = Node(
        package="mesh_splat",
        executable="projective_mesher_node",
        name="projective_mesher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "mesh_frame_id": LaunchConfiguration("mesh_frame_id"),
                "max_triangle_side": LaunchConfiguration("max_triangle_side"),
                "min_ratio_btw_largest_smallest_side": LaunchConfiguration(
                    "min_ratio_btw_largest_smallest_side"
                ),
                "min_observations": LaunchConfiguration("min_observations"),
                "enable_rerun": LaunchConfiguration("enable_rerun"),
                "rerun_host": LaunchConfiguration("rerun_host"),
                "rerun_app_id": LaunchConfiguration("rerun_app_id"),
                "rerun_recording_id": LaunchConfiguration("rerun_recording_id"),
                "rerun_entity_path": LaunchConfiguration("rerun_entity_path"),
                "rerun_camera_entity_path": LaunchConfiguration(
                    "rerun_camera_entity_path"
                ),
                "log_rerun_camera_pose": LaunchConfiguration("log_rerun_camera_pose"),
            }
        ],
        remappings=[
            ("mesher_input", LaunchConfiguration("mesher_input_topic")),
            ("mesh", LaunchConfiguration("mesh_topic")),
            ("debug_mesh_img", LaunchConfiguration("texture_topic")),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            mesher_input_topic_arg,
            mesh_topic_arg,
            texture_topic_arg,
            mesh_frame_id_arg,
            max_triangle_side_arg,
            min_side_ratio_arg,
            min_observations_arg,
            enable_rerun_arg,
            rerun_host_arg,
            rerun_app_id_arg,
            rerun_recording_id_arg,
            rerun_entity_path_arg,
            rerun_camera_entity_path_arg,
            log_rerun_camera_pose_arg,
            mesher_node,
        ]
    )
