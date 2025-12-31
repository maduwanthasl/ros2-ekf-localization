from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("ekf_miniproject")
    xacro_file = os.path.join(pkg, "urdf", "four_wheel_bot.urdf.xacro")
    rviz_config = os.path.join(pkg, "rviz", "ekf_demo.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        # Static transform: map -> base_link (identity for now, robot moves in map frame)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
            output="screen"
        ),
        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        ),

        # Simulator publishes: /truth/pose, /gps, /imu_yaw, /cmd_vel
        Node(
            package="ekf_miniproject",
            executable="sim_node",
            output="screen"
        ),

        # EKF subscribes and publishes: /ekf/pose, /ekf/path
        Node(
            package="ekf_miniproject",
            executable="ekf_node",
            output="screen"
        ),

        # RViz - open manually if auto-launch fails:
        # rviz2 -d /home/shevi/ekf_ws/install/ekf_miniproject/share/ekf_miniproject/rviz/ekf_demo.rviz
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     arguments=["-d", rviz_config],
        #     output="screen"
        # ),
    ])
