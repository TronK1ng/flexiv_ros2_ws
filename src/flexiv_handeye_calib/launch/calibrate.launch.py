"""Bring up the wrist camera (optionally) and the hand-eye calibration node.

Typical use, with the robot stack and the camera already running:

    ros2 launch flexiv_handeye_calib calibrate.launch.py \
        robot_sn:=Rizon4s-123456 launch_bringup:=false start_camera:=false

Then move the arm to ~15 different orientations with the marker in view and call

    ros2 service call /handeye_calibrator/capture std_srvs/srv/Trigger
    ros2 service call /handeye_calibrator/solve std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bool(value):
    return value.strip().lower() in ("true", "1", "yes")


def _setup(context, *args, **kwargs):
    robot_sn = LaunchConfiguration("robot_sn").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    if not robot_sn:
        raise RuntimeError("robot_sn is required, e.g. robot_sn:=Rizon4s-123456")

    config_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_handeye_calib"), "config", "handeye.yaml"]
    )
    nodes = []

    if _bool(LaunchConfiguration("launch_bringup").perform(context)):
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("flexiv_bringup"), "launch", "rizon_moveit.launch.py"]
                    )
                ),
                launch_arguments={
                    "robot_sn": robot_sn,
                    "robot_type": robot_type,
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
                    "start_rviz": LaunchConfiguration("start_rviz").perform(context),
                }.items(),
            )
        )

    if _bool(LaunchConfiguration("start_camera").perform(context)):
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                    )
                ),
                launch_arguments={
                    "enable_color": "true",
                    "enable_depth": "false",
                    "rgb_camera.color_profile": LaunchConfiguration("color_profile").perform(context),
                    "pointcloud.enable": "false",
                    "align_depth.enable": "false",
                }.items(),
            )
        )

    parameters = {
        "robot_sn": robot_sn,
        "camera_frame": LaunchConfiguration("camera_frame").perform(context),
        "image_topic": LaunchConfiguration("image_topic").perform(context),
        "camera_info_topic": LaunchConfiguration("camera_info_topic").perform(context),
        "target_type": LaunchConfiguration("target_type").perform(context),
        "tag_family": LaunchConfiguration("tag_family").perform(context),
        "tag_size": float(LaunchConfiguration("tag_size").perform(context)),
        "tag_id": int(LaunchConfiguration("tag_id").perform(context)),
        "chessboard_cols": int(LaunchConfiguration("chessboard_cols").perform(context)),
        "chessboard_rows": int(LaunchConfiguration("chessboard_rows").perform(context)),
        "square_size": float(LaunchConfiguration("square_size").perform(context)),
        "chessboard_auto": _bool(LaunchConfiguration("chessboard_auto").perform(context)),
        "auto_capture": _bool(LaunchConfiguration("auto_capture").perform(context)),
        "solve_method": LaunchConfiguration("solve_method").perform(context),
    }
    for name in ("samples_file", "output_yaml"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = value

    nodes.append(
        Node(
            package="flexiv_handeye_calib",
            executable="calibrate",
            name="handeye_calibrator",
            parameters=[config_file, parameters],
            output="screen",
        )
    )
    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("robot_sn", description="e.g. Rizon4s-123456"),
        DeclareLaunchArgument("robot_type", default_value="Rizon4s"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/camera/camera/color/camera_info"
        ),
        DeclareLaunchArgument("color_profile", default_value="1280x720x30"),
        DeclareLaunchArgument("target_type", default_value="chessboard",
                              description="apriltag or chessboard"),
        DeclareLaunchArgument("tag_family", default_value="25h9"),
        DeclareLaunchArgument("tag_size", default_value="0.026",
                              description="black square side in metres, including its black border"),
        DeclareLaunchArgument("tag_id", default_value="-1", description="-1 picks the largest tag"),
        DeclareLaunchArgument("chessboard_cols", default_value="8",
                              description="inner corners per row"),
        DeclareLaunchArgument("chessboard_rows", default_value="11",
                              description="inner corners per column"),
        DeclareLaunchArgument("square_size", default_value="0.006",
                              description="chessboard square side in metres"),
        DeclareLaunchArgument("chessboard_auto", default_value="true"),
        DeclareLaunchArgument("auto_capture", default_value="false"),
        DeclareLaunchArgument("solve_method", default_value="PARK"),
        DeclareLaunchArgument("samples_file", default_value=""),
        DeclareLaunchArgument("output_yaml", default_value=""),
        DeclareLaunchArgument("launch_bringup", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup)])
