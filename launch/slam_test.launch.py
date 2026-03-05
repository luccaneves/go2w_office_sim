"""
GO2W in office world with SLAM Toolbox only (no Nav2) for mapping tests.
Use keyboard teleop in another terminal to drive the robot.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_office_sim")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    use_rviz = LaunchConfiguration("use_rviz")

    xacro_file = os.path.join(pkg_dir, "urdf", "go2w_gz.urdf.xacro")
    world_file = os.path.join(pkg_dir, "worlds", "office.sdf")
    slam_params_file = os.path.join(pkg_dir, "config", "slam_params.yaml")
    laser_filter_file = os.path.join(pkg_dir, "config", "laser_filter.yaml")
    rviz_config_file = os.path.join(pkg_dir, "rviz", "nav2.rviz")

    declare_use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")

    stdout_linebuf = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command([FindExecutable(name="xacro"), " ", xacro_file]),
                    value_type=str,
                ),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "go2w",
            "-x", "0", "-y", "0", "-z", "0.5",
        ],
        output="screen",
    )

    # ROS-GZ Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen",
    )

    # Laser scan filter (removes robot self-occlusion)
    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[laser_filter_file],
        remappings=[
            ("scan", "/scan_raw"),
            ("scan_filtered", "/scan"),
        ],
        output="screen",
    )

    # SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # Info
    teleop_info = LogInfo(
        msg="\n\n========================================\n"
        "  GO2W SLAM Test (no Nav2)\n"
        "  - Drive with keyboard teleop:\n"
        "    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n"
        "  - Check /scan: ros2 topic echo /scan --once\n"
        "  - Check TF:    ros2 run tf2_ros tf2_echo map base\n"
        "\n========================================\n"
    )

    return LaunchDescription(
        [
            stdout_linebuf,
            declare_use_rviz,
            gz_sim,
            robot_state_publisher,
            spawn_robot,
            bridge,
            scan_filter,
            slam_toolbox,
            rviz_node,
            teleop_info,
        ]
    )
