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

    urdf = os.path.join(
        get_package_share_directory("insertion_robot_description"),
        "urdf", "insertion_robot.urdf")
    with open(urdf, 'r') as infp:
        robot_description = infp.read()


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('insertion_robot_description'),
            "controller",
            'insertion_robot_upload.yaml',
        ]
    )


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

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )
    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'],
    #     output='screen'
    # )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=['joint_trajectory_controller', "-c", "/controller_manager"],
    )


    nodes_to_start = [
        # joint_state_publisher_gui_node,
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=control_node,
                on_exit=[robot_state_publisher_node],
            )
        ),
        joint_state_publisher_node,
        # robot_state_publisher_node,
        rviz_node,
        control_node,
        # load_joint_trajectory_controller,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(nodes_to_start)