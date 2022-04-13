from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

pkg_insertion_robot             = FindPackageShare('needle_insertion_robot')
pkg_insertion_robot_translation = FindPackageShare('needle_insertion_robot_translation')

def generate_launch_description():
    ld = LaunchDescription()

    # arguments
    arg_namespace = DeclareLaunchArgument(
        'ns',
        default_value="stage",
        description="ROS namespace for the robot launching"
    )
    
    # - robot
    arg_robot_ip = DeclareLaunchArgument(
        'robot_ipAddress',
        default_value="192.168.1.201",
        description="Robot: IP Address of Galil Controller"
    )
    

    # launch files
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_insertion_robot, 'launch', 'robot.launch.py'
            ])
        ),
        launch_arguments={
            'ip': LaunchConfiguration('robot_ipAddress'),
            'ns': LaunchConfiguration('ns'),
        }.items(),
    )

    launch_translation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_insertion_robot_translation, 'launch', 'robot_translation.launch.py'
            ])
        ),
        launch_arguments={
            'ns': LaunchConfiguration('ns'),
        }.items(),
    )

    # launch description setup
    # - arguments
    ld.add_action( arg_namespace )
    ld.add_action( arg_robot_ip )

    # - launch files
    ld.add_action( launch_robot )
    ld.add_action( launch_translation )


    return ld

# generate_launch_description
