import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("go2w_office_sim")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "go2w_gz.urdf.xacro"])
    world_file = PathJoinSubstitution([pkg_share, "worlds", "office.sdf"])

    # Robot state publisher
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

    # Launch Gazebo Sim (gz-sim) with office world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
    )

    # Spawn robot into gz-sim from robot_description topic
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "go2w",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.5",
        ],
        output="screen",
    )

    # Bridge: cmd_vel (ROS->GZ), odom (GZ->ROS), clock (GZ->ROS),
    #         joint_states (GZ->ROS), scan (GZ->ROS), imu (GZ->ROS)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen",
    )

    # Print instructions for teleop
    teleop_info = LogInfo(
        msg="\n\n========================================\n"
        "  Gazebo simulation is launching!\n"
        "  To control the robot with keyboard, run in another terminal:\n\n"
        '    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
        "\n========================================\n"
    )

    return LaunchDescription(
        [
            gz_sim,
            robot_state_publisher,
            spawn_robot,
            bridge,
            teleop_info,
        ]
    )
