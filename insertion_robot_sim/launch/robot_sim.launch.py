import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    robot_sim_node = Node(
        package="insertion_robot_sim",
        executable="robot_sim",
    )

    # use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')

    
    # declare_simulator_cmd = DeclareLaunchArgument(
    # name='headless',
    # default_value='False',
    # description='Whether to execute gzclient')
    
    # declare_use_sim_time_cmd = DeclareLaunchArgument(
    # name='use_sim_time',
    # default_value='true',
    # description='Use simulation (Gazebo) clock if true')
 
    # declare_use_simulator_cmd = DeclareLaunchArgument(
    # name='use_simulator',
    # default_value='True',
    # description='Whether to start the simulator')

    # Loading robot_description
    upload_robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('insertion_robot_description'), 'launch', 'insertion_robot_upload.launch.py')),
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        # condition=IfCondition(use_simulator),

    )
 
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        # condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'insertionRobot'],
                        output='screen')

    nodes_to_start = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[upload_robot_description],
            )
        ),
        upload_robot_description,
        robot_sim_node,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        
    ]
    
    ld = LaunchDescription(nodes_to_start)
    # ld.add_action(declare_simulator_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_use_simulator_cmd)
    return ld