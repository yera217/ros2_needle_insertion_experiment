import os
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name="insertion_robot_description"

    urdf = os.path.join(
        get_package_share_directory(package_name),
        "urdf", "insertion_robot.urdf")
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    robot_description = {"robot_description": robot_description}


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "controller",
            'insertion_robot_controller.yaml',
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "worlds", "insertion_robot.rviz"]
    )



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
        output="screen",
        parameters=[robot_description],
        arguments=[urdf],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        arguments=[urdf],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=['position_controllers'],
    )


    nodes_to_start = [
        # joint_state_publisher_gui_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(nodes_to_start)