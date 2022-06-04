import os
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    urdf = os.path.join(
        get_package_share_directory("insertion_robot_description"),
        "urdf", "insertion_robot.urdf")
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    # )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        arguments=[urdf],
    )
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    # )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=[urdf],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        # joint_state_publisher_gui_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)