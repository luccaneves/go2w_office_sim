"""
GO2W in office world with SLAM + Nav2 + RViz.
  - gz-sim with office.sdf
  - GO2W robot (DiffDrive wheels, fixed legs)
  - SLAM Toolbox (online_async)
  - Nav2 (MPPI controller, SmacPlanner2D)
  - RViz for visualization and 2D Nav Goal
  - Keyboard teleop via separate terminal
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2w_office_sim")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    # File paths
    xacro_file = os.path.join(pkg_dir, "urdf", "go2w_gz.urdf.xacro")
    world_file = os.path.join(pkg_dir, "worlds", "office.sdf")
    nav2_params_file = os.path.join(pkg_dir, "config", "nav2_params.yaml")
    slam_params_file = os.path.join(pkg_dir, "config", "slam_params.yaml")
    laser_filter_file = os.path.join(pkg_dir, "config", "laser_filter.yaml")
    rviz_config_file = os.path.join(pkg_dir, "rviz", "nav2.rviz")

    # Nav2 params with autostart
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key="",
            param_rewrites={"autostart": "true"},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # ===== Launch arguments =====
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true"
    )

    stdout_linebuf = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # ===== 1. Gazebo Sim =====
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

    # ===== 2. Robot State Publisher =====
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

    # ===== 3. Spawn robot =====
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

    # ===== 4. ROS-GZ Bridge (3D pointcloud from Gazebo) =====
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/pointcloud_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen",
    )

    # ===== 4b. 3D pointcloud -> 2D laser scan =====
    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[{
            "target_frame": "lidar",
            "transform_tolerance": 0.05,
            "min_height": -2.0,
            "max_height": 2.0,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.00872665,
            "scan_time": 0.1,
            "range_min": 0.05,
            "range_max": 30.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
        }],
        remappings=[
            ("cloud_in", "/pointcloud_raw/points"),
            ("scan", "/scan_raw"),
        ],
        output="screen",
    )

    # ===== 4c. Laser scan filter (removes robot self-occlusion) =====
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

    # ===== 5. SLAM Toolbox =====
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # ===== 6. Nav2 Stack =====
    nav2_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
    ]

    nav2_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": nav2_lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # ===== 7. RViz =====
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # ===== 8. Info =====
    teleop_info = LogInfo(
        msg="\n\n========================================\n"
        "  GO2W Nav2 Office Simulation\n"
        "  - Use RViz '2D Nav Goal' to send navigation goals\n"
        "  - Or run keyboard teleop in another terminal:\n"
        "    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n"
        "\n========================================\n"
    )

    return LaunchDescription(
        [
            stdout_linebuf,
            declare_use_sim_time,
            declare_use_rviz,
            gz_sim,
            robot_state_publisher,
            spawn_robot,
            bridge,
            pointcloud_to_scan,
            scan_filter,
            slam_toolbox,
            TimerAction(period=10.0, actions=[nav2_nodes]),
            rviz_node,
            teleop_info,
        ]
    )
