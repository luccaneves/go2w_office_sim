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
    #         joint_states (GZ->ROS), pointcloud (GZ->ROS), imu (GZ->ROS)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/pointcloud_raw@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen",
    )

    # Convert 3D pointcloud to 2D laser scan (simulates L2 4D LiDAR -> 2D)
    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[
            {
                "use_sim_time": True,
                "target_frame": "lidar",
                "min_height": -0.1,
                "max_height": 0.3,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.00872665,  # ~720 points
                "scan_time": 0.1,
                "range_min": 0.05,
                "range_max": 30.0,
                "inf_epsilon": 1.0,
                "use_inf": True,
            }
        ],
        remappings=[
            ("cloud_in", "/pointcloud_raw"),
            ("scan", "/scan"),
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
            pointcloud_to_scan,
            teleop_info,
        ]
    )
