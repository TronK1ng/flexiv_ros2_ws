"""One-shot launch for the grasp demo.

Starts, in this order: the RealSense camera, the robot stack (driver + ros_control +
move_group + RViz), the hand-eye TF, a fake gripper stand-in when running with fake
hardware, the perception node when detect_mode is "vision", and finally the task node.

Defaults below are wired to the lab robot (Rizon4s-064230) at 5% speed, so the plain
`ros2 launch flexiv_grasp_demo grasp_demo.launch.py` command is the normal way to run it.

The robot stack can be skipped with launch_bringup:=false when it is already
running elsewhere.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return value.strip().lower() in ("true", "1", "yes")


def _resolve_handeye_yaml(candidate):
    """Return the calibration file to use, or None if there is none.

    An explicit path must exist; otherwise the usual locations are probed, with
    the copy shipped in this package as the last resort.
    """
    if candidate:
        if not os.path.isfile(candidate):
            raise RuntimeError(f"handeye_yaml '{candidate}' does not exist")
        return candidate

    for path in (
        os.path.join(os.getcwd(), ".cache", "handeye", "handeye_calibration.yaml"),
        os.path.expanduser("~/.cache/handeye/handeye_calibration.yaml"),
        os.path.join(
            get_package_share_directory("flexiv_grasp_demo"), "config", "handeye_calibration.yaml"
        ),
    ):
        if os.path.isfile(path):
            return path
    return None


def _load_yaml(package_name, file_path, replacements=None):
    path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    for key, value in (replacements or {}).items():
        text = text.replace(key, value)
    import yaml

    return yaml.safe_load(text)


def _launch_setup(context, *args, **kwargs):
    robot_sn = LaunchConfiguration("robot_sn").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    detect_mode = LaunchConfiguration("detect_mode").perform(context)
    launch_bringup = _as_bool(LaunchConfiguration("launch_bringup").perform(context))
    use_fake = _as_bool(LaunchConfiguration("use_fake_hardware").perform(context))

    if not robot_sn:
        raise RuntimeError("robot_sn is required, e.g. robot_sn:=Rizon4s-123456")

    start_camera = _as_bool(LaunchConfiguration("start_camera").perform(context))
    camera_color_profile = LaunchConfiguration("camera_color_profile").perform(context)

    config_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_grasp_demo"), "config", "grasp_demo.yaml"]
    )
    base_frame = f"{robot_sn}_base_link"

    nodes = []

    if start_camera:
        # 1280x720 roughly doubles the marker's pixel size compared with 640x480,
        # which is what keeps a 26 mm AprilTag detectable at 0.3-0.4 m.
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                    )
                ),
                launch_arguments={
                    "rgb_camera.color_profile": camera_color_profile,
                }.items(),
            )
        )

    if launch_bringup:
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
                    "use_fake_hardware": "true" if use_fake else "false",
                    "load_gripper": LaunchConfiguration("load_gripper").perform(context),
                    # RViz is started below with the demo's own view config.
                    "start_rviz": "false",
                    "rdk_control_mode": LaunchConfiguration("robot_control_mode").perform(context),
                    "kinematics_params_file": LaunchConfiguration("kinematics_params_file").perform(
                        context
                    ),
                }.items(),
            )
        )

    if _as_bool(LaunchConfiguration("start_rviz").perform(context)):
        rviz_config = LaunchConfiguration("rviz_config").perform(context)
        if not rviz_config:
            rviz_config = os.path.join(
                get_package_share_directory("flexiv_grasp_demo"), "rviz", "moveit_view.rviz"
            )
        if not os.path.isfile(rviz_config):
            raise RuntimeError(f"rviz_config '{rviz_config}' does not exist")

        # The MotionPlanning display (the interactive 6-DoF marker) needs the model on
        # the RViz node itself: it does not pick it up from the robot_description topic
        # in this MoveIt version, and without it the marker never appears.
        load_gripper_arg = LaunchConfiguration("load_gripper").perform(context)
        kinematics_params = LaunchConfiguration("kinematics_params_file").perform(context)
        kinematics_xacro_arg = (
            f" kinematics_parameters_file:={kinematics_params}" if kinematics_params else ""
        )
        robot_description = ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("flexiv_hardware"), "urdf", "flexiv.urdf.xacro"]
                    ),
                    " robot_sn:=", robot_sn,
                    " robot_type:=", robot_type,
                    " ros2_control:=true",
                    " rdk_control_mode:=",
                    LaunchConfiguration("robot_control_mode").perform(context),
                    " load_gripper:=", load_gripper_arg,
                    " gripper_name:=Flexiv-GN01",
                    " load_mounted_ft_sensor:=false",
                    " use_fake_hardware:=", "true" if use_fake else "false",
                    " fake_sensor_commands:=false",
                    kinematics_xacro_arg,
                ]
            ),
            value_type=str,
        )
        robot_description_semantic = ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("flexiv_moveit_config"), "srdf", "rizon.srdf.xacro"]
                    ),
                    " robot_sn:=", robot_sn,
                    " robot_type:=", robot_type,
                    " load_gripper:=", load_gripper_arg,
                    " load_mounted_ft_sensor:=false",
                ]
            ),
            value_type=str,
        )
        robot_description_kinematics = _load_yaml(
            "flexiv_moveit_config",
            "config/kinematics.yaml",
            {"$(var robot_sn)": robot_sn},
        )
        # Start RViz a few seconds late so it initialises after move_group has published
        # the planning scene; otherwise its scene monitor can latch onto a zero robot state.
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[
                {
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                    "robot_description_kinematics": robot_description_kinematics,
                }
            ],
            output="log",
        )
        nodes.append(TimerAction(period=6.0, actions=[rviz_node]))

    if _as_bool(LaunchConfiguration("publish_camera_tf").perform(context)):
        calibration = _resolve_handeye_yaml(
            LaunchConfiguration("handeye_yaml").perform(context)
        )
        if calibration is None:
            raise RuntimeError(
                "no hand-eye calibration found; pass handeye_yaml:=<path> or set "
                "publish_camera_tf:=false"
            )
        nodes.append(
            Node(
                package="flexiv_grasp_demo",
                executable="handeye_tf_publisher.py",
                name="handeye_tf_publisher",
                parameters=[{"yaml_path": calibration, "robot_sn": robot_sn}],
                output="screen",
            )
        )

    if use_fake:
        nodes.append(
            Node(
                package="flexiv_grasp_demo",
                executable="fake_gripper_server.py",
                name="flexiv_gripper_node",
                parameters=[config_file, {"joint_name": f"{robot_sn}_finger_width_joint"}],
                output="screen",
            )
        )

    if detect_mode == "vision":
        nodes.append(
            Node(
                package="flexiv_grasp_demo",
                executable="grasp_perception.py",
                name="grasp_perception",
                parameters=[config_file, {"base_frame": base_frame}],
                output="screen",
            )
        )

    if not _as_bool(LaunchConfiguration("start_task").perform(context)):
        return nodes

    task_parameters = {
        "robot_sn": robot_sn,
        "detect_mode": detect_mode,
        "base_frame": base_frame,
    }
    for name in ("velocity_scaling", "acceleration_scaling", "grasp_force"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            task_parameters[name] = float(value)

    nodes.append(
        Node(
            package="flexiv_grasp_demo",
            executable="grasp_task_node",
            name="grasp_task",
            parameters=[config_file, task_parameters],
            output="screen",
        )
    )

    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "robot_sn",
            default_value="Rizon4s-064230",
            description="Robot serial number, hyphens kept, e.g. Rizon4s-123456",
        ),
        DeclareLaunchArgument(
            "robot_type",
            default_value="Rizon4s",
            description="Flexiv robot type; must match the physical robot",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Run the whole stack without a robot and use the fake gripper server",
        ),
        DeclareLaunchArgument(
            "detect_mode",
            default_value="vision",
            description="'vision' uses the AprilTag pipeline, 'mock' uses mock_tag_xyz from the YAML",
        ),
        DeclareLaunchArgument(
            "launch_bringup",
            default_value="true",
            description="Start the robot stack (driver, move_group, RViz) from this launch file",
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="true",
            description="Load the Grav gripper model; needed for the grav TCP frame",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz with the demo view (see rviz_config)",
        ),
        DeclareLaunchArgument(
            "robot_control_mode",
            default_value="joint_position",
            description="RDK control mode for the joint position interface",
        ),
        DeclareLaunchArgument(
            "velocity_scaling",
            default_value="0.05",
            description="Task node velocity scaling; 0.05 is the safe lab value",
        ),
        DeclareLaunchArgument(
            "acceleration_scaling",
            default_value="0.05",
            description="Task node acceleration scaling",
        ),
        DeclareLaunchArgument(
            "grasp_force",
            default_value="10.0",
            description="Gripper grasp force in N (20 g cube)",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="RViz config for the demo view; empty uses the package's rviz/moveit_view.rviz",
        ),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Start the RealSense camera from this launch file",
        ),
        DeclareLaunchArgument(
            "camera_color_profile",
            default_value="1280x720x30",
            description="RealSense colour stream profile, e.g. 1280x720x30 or 640x480x30",
        ),
        DeclareLaunchArgument(
            "start_task",
            default_value="true",
            description="Start the task node; disable to run it separately with custom parameters",
        ),
        DeclareLaunchArgument(
            "publish_camera_tf",
            default_value="true",
            description="Publish flange -> camera_link from the hand-eye calibration",
        ),
        DeclareLaunchArgument(
            "handeye_yaml",
            default_value="",
            description=(
                "Hand-eye calibration YAML; empty probes ./.cache/handeye, ~/.cache/handeye "
                "and the copy shipped with this package"
            ),
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value="",
            description="Per-robot kinematics YAML from flexiv_calibration; empty uses the nominal one",
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
