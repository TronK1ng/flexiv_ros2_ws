"""Run the validator against the calibration currently published on tf.

    ros2 launch flexiv_handeye_calib validate.launch.py robot_sn:=Rizon4s-123456
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    robot_sn = LaunchConfiguration("robot_sn").perform(context)
    if not robot_sn:
        raise RuntimeError("robot_sn is required, e.g. robot_sn:=Rizon4s-123456")
    config = PathJoinSubstitution(
        [FindPackageShare("flexiv_handeye_calib"), "config", "handeye.yaml"]
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
    }
    report = LaunchConfiguration("report_file").perform(context)
    if report:
        parameters["report_file"] = report
    return [
        Node(
            package="flexiv_handeye_calib",
            executable="validate",
            name="handeye_validator",
            parameters=[config, parameters],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_sn"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
            DeclareLaunchArgument("target_type", default_value="chessboard"),
            DeclareLaunchArgument("tag_family", default_value="25h9"),
            DeclareLaunchArgument("chessboard_cols", default_value="8"),
            DeclareLaunchArgument("chessboard_rows", default_value="11"),
            DeclareLaunchArgument("square_size", default_value="0.006"),
            DeclareLaunchArgument("tag_size", default_value="0.026"),
            DeclareLaunchArgument("tag_id", default_value="-1"),
            DeclareLaunchArgument("report_file", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
