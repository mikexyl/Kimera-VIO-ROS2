import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _prepare_result_dir(context, *_args, **_kwargs):
    result_dir = LaunchConfiguration("rerun_result_dir").perform(context)
    if result_dir:
        os.makedirs(result_dir, exist_ok=True)
    return []


def generate_launch_description():
    dataset_arg = DeclareLaunchArgument(
        "dataset_name",
        default_value="Euroc",
        description="Name of the dataset whose parameters should be used.",
    )
    parallel_arg = DeclareLaunchArgument(
        "parallel",
        default_value="false",
        description="Unused placeholder kept for parity with the ROS1 launch file.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true.",
    )
    log_output_arg = DeclareLaunchArgument(
        "log_output",
        default_value="false",
        description="Enable logging to the output path.",
    )
    log_output_path_arg = DeclareLaunchArgument(
        "log_output_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kimera_vio_ros"), "output_logs"]
        ),
        description="Directory where module logs should be stored.",
    )
    use_lcd_arg = DeclareLaunchArgument(
        "use_lcd",
        default_value="0",
        description="Enable the loop-closure detector if true.",
    )
    workspace_root_arg = DeclareLaunchArgument(
        "workspace_root",
        default_value=EnvironmentVariable(
            "ROS_WS", default_value="/home/mikexyl/workspaces/kimera_ros2_ws"
        ),
        description="Workspace root used to locate non-ament Kimera assets.",
    )
    params_folder_arg = DeclareLaunchArgument(
        "params_folder",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("kimera_vio_ros"),
                "param",
                LaunchConfiguration("dataset_name"),
            ]
        ),
        description="Directory that contains Kimera-VIO parameter files.",
    )
    path_to_vocab_arg = DeclareLaunchArgument(
        "path_to_vocab",
        default_value=PathJoinSubstitution(
            [
                LaunchConfiguration("workspace_root"),
                "src",
                "Kimera-VIO-ROS2",
                "MIT-SPARK",
                "Kimera-VIO",
                "vocabulary",
                "ORBvoc.yml",
            ]
        ),
        description="Absolute path to the ORB vocabulary file.",
    )
    topic_left_image_arg = DeclareLaunchArgument(
        "topic.left.image",
        default_value="/cam0/image_raw",
        description="Left camera image topic.",
    )
    topic_right_image_arg = DeclareLaunchArgument(
        "topic.right.image",
        default_value="/cam1/image_raw",
        description="Right camera image topic.",
    )
    topic_imu_data_arg = DeclareLaunchArgument(
        "topic.imu.data",
        default_value="/imu0",
        description="IMU topic.",
    )
    use_camera_info_arg = DeclareLaunchArgument(
        "use_camera_info",
        default_value="false",
        description="Subscribe to camera info topics if true.",
    )
    topic_left_info_arg = DeclareLaunchArgument(
        "topic.left.info",
        default_value="/cam0/camera_info",
        description="Left camera info topic.",
    )
    topic_right_info_arg = DeclareLaunchArgument(
        "topic.right.info",
        default_value="/cam1/camera_info",
        description="Right camera info topic.",
    )
    frame_id_base_link_arg = DeclareLaunchArgument(
        "frame_id.base_link",
        default_value="base_link",
        description="Base link frame id.",
    )
    frame_id_map_arg = DeclareLaunchArgument(
        "frame_id.map",
        default_value="map",
        description="Map frame id.",
    )
    frame_id_world_arg = DeclareLaunchArgument(
        "frame_id.world",
        default_value="world",
        description="World frame id.",
    )
    verbosity_arg = DeclareLaunchArgument(
        "verbosity",
        default_value="0",
        description="Glog verbosity level.",
    )
    visualize_arg = DeclareLaunchArgument(
        "visualize",
        default_value="true",
        description="Enable OpenCV visualizations if true.",
    )
    rerun_host_arg = DeclareLaunchArgument(
        "rerun_host",
        default_value="rerun+http://127.0.0.1:9876/proxy",
        description="Rerun gRPC endpoint used by the standalone mesher.",
    )
    rerun_app_id_arg = DeclareLaunchArgument(
        "rerun_app_id",
        default_value="kimera_vio",
        description="Rerun application id. Keep kimera_vio to overlay with Kimera.",
    )
    rerun_recording_id_arg = DeclareLaunchArgument(
        "rerun_recording_id",
        default_value="kimera_projective_mesher_comparison",
        description="Rerun recording id shared by Kimera and the standalone mesher.",
    )
    rerun_result_dir_arg = DeclareLaunchArgument(
        "rerun_result_dir",
        default_value="",
        description="Optional Kimera Rerun side-output directory.",
    )
    standalone_mesh_topic_arg = DeclareLaunchArgument(
        "standalone_mesh_topic",
        default_value="/standalone_projective_mesher/mesh",
        description="Standalone mesher PolygonMesh output topic.",
    )
    standalone_texture_topic_arg = DeclareLaunchArgument(
        "standalone_texture_topic",
        default_value="/standalone_projective_mesher/debug_mesh_img",
        description="Standalone mesher texture image output topic.",
    )
    standalone_entity_path_arg = DeclareLaunchArgument(
        "standalone_entity_path",
        default_value="map/standalone_projective_mesher/mesh",
        description="Rerun entity path for the standalone mesher output.",
    )
    standalone_camera_entity_path_arg = DeclareLaunchArgument(
        "standalone_camera_entity_path",
        default_value="map/standalone_projective_mesher/camera",
        description="Rerun entity path for the standalone mesher input camera pose.",
    )
    standalone_max_triangle_side_arg = DeclareLaunchArgument(
        "standalone_max_triangle_side",
        default_value="2.0",
        description="Standalone mesher maximum triangle side length. <=0 disables.",
    )
    standalone_min_side_ratio_arg = DeclareLaunchArgument(
        "standalone_min_ratio_btw_largest_smallest_side",
        default_value="0.2",
        description="Standalone mesher min_side/max_side rejection threshold.",
    )
    standalone_min_observations_arg = DeclareLaunchArgument(
        "standalone_min_observations",
        default_value="3",
        description="Minimum valid observations required by the standalone mesher.",
    )
    play_bag_arg = DeclareLaunchArgument(
        "play_bag",
        default_value="false",
        description="Play a rosbag as part of the launch if true.",
    )
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value="",
        description="Absolute path to the rosbag to play when play_bag is true.",
    )
    bag_play_delay_arg = DeclareLaunchArgument(
        "bag_play_delay",
        default_value="3.0",
        description="Delay in seconds before starting rosbag playback.",
    )
    bag_clock_hz_arg = DeclareLaunchArgument(
        "bag_clock_hz",
        default_value="100.0",
        description="Clock publish frequency passed to ros2 bag play --clock.",
    )

    kimera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("kimera_vio_ros"), "launch", "kimera_vio_ros.launch.py"]
            )
        ),
        launch_arguments={
            "dataset_name": LaunchConfiguration("dataset_name"),
            "parallel": LaunchConfiguration("parallel"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "log_output": LaunchConfiguration("log_output"),
            "log_output_path": LaunchConfiguration("log_output_path"),
            "use_lcd": LaunchConfiguration("use_lcd"),
            "workspace_root": LaunchConfiguration("workspace_root"),
            "params_folder": LaunchConfiguration("params_folder"),
            "path_to_vocab": LaunchConfiguration("path_to_vocab"),
            "topic.left.image": LaunchConfiguration("topic.left.image"),
            "topic.right.image": LaunchConfiguration("topic.right.image"),
            "topic.imu.data": LaunchConfiguration("topic.imu.data"),
            "use_camera_info": LaunchConfiguration("use_camera_info"),
            "topic.left.info": LaunchConfiguration("topic.left.info"),
            "topic.right.info": LaunchConfiguration("topic.right.info"),
            "frame_id.base_link": LaunchConfiguration("frame_id.base_link"),
            "frame_id.map": LaunchConfiguration("frame_id.map"),
            "frame_id.world": LaunchConfiguration("frame_id.world"),
            "verbosity": LaunchConfiguration("verbosity"),
            "visualize": LaunchConfiguration("visualize"),
            "viz_type": "0",
            "publish_mesher_input": "true",
            "rerun_recording_id": LaunchConfiguration("rerun_recording_id"),
            "rerun_result_dir": LaunchConfiguration("rerun_result_dir"),
        }.items(),
    )

    standalone_mesher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mesh_splat"),
                    "launch",
                    "projective_mesher.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "mesher_input_topic": "/kimera_vio_ros/mesher_input",
            "mesh_topic": LaunchConfiguration("standalone_mesh_topic"),
            "texture_topic": LaunchConfiguration("standalone_texture_topic"),
            "max_triangle_side": LaunchConfiguration("standalone_max_triangle_side"),
            "min_ratio_btw_largest_smallest_side": LaunchConfiguration(
                "standalone_min_ratio_btw_largest_smallest_side"
            ),
            "min_observations": LaunchConfiguration("standalone_min_observations"),
            "enable_rerun": "true",
            "rerun_host": LaunchConfiguration("rerun_host"),
            "rerun_app_id": LaunchConfiguration("rerun_app_id"),
            "rerun_recording_id": LaunchConfiguration("rerun_recording_id"),
            "rerun_entity_path": LaunchConfiguration("standalone_entity_path"),
            "rerun_camera_entity_path": LaunchConfiguration(
                "standalone_camera_entity_path"
            ),
            "log_rerun_camera_pose": "true",
        }.items(),
    )

    bag_play = TimerAction(
        period=LaunchConfiguration("bag_play_delay"),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    LaunchConfiguration("bag_path"),
                    "--clock",
                    LaunchConfiguration("bag_clock_hz"),
                    "--disable-keyboard-controls",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("play_bag")),
            )
        ],
    )

    return LaunchDescription(
        [
            dataset_arg,
            parallel_arg,
            use_sim_time_arg,
            log_output_arg,
            log_output_path_arg,
            use_lcd_arg,
            workspace_root_arg,
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
            rerun_host_arg,
            rerun_app_id_arg,
            rerun_recording_id_arg,
            rerun_result_dir_arg,
            standalone_mesh_topic_arg,
            standalone_texture_topic_arg,
            standalone_entity_path_arg,
            standalone_camera_entity_path_arg,
            standalone_max_triangle_side_arg,
            standalone_min_side_ratio_arg,
            standalone_min_observations_arg,
            play_bag_arg,
            bag_path_arg,
            bag_play_delay_arg,
            bag_clock_hz_arg,
            OpaqueFunction(function=_prepare_result_dir),
            kimera_launch,
            standalone_mesher_launch,
            bag_play,
        ]
    )
