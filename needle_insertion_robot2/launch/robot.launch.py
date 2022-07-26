from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    package_name = 'needle_insertion_robot'

    ld = LaunchDescription()

    # arguments
    ros_paramfile_arg = DeclareLaunchArgument( 'paramFile',
                                               default_value='default_params.yaml',
                                               description='Parameter file to use in local share directory. This overrides all arguments.' 
                                               )
    robot_ip_arg = DeclareLaunchArgument( 'ip',
                                          default_value='192.168.1.201',
                                          description="IP Address of Galil Controller."
                                          )
    namespace_arg = DeclareLaunchArgument('ns', default_value='',description="ROS Namespace for robot node.")

    # Nodes
    robot_node = Node(
            package=package_name,
            namespace=LaunchConfiguration('ns'),
            executable='insertion_robot_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'robot.ip_address': LaunchConfiguration('ip'),
            },
            PathJoinSubstitution([FindPackageShare(package_name),
                                  'config',
                                  LaunchConfiguration('paramFile')
                                  ]
                                )
            ]
    )

    
    # add to launch description
    ld.add_action( ros_paramfile_arg )
    ld.add_action( robot_ip_arg )
    ld.add_action( namespace_arg )

    ld.add_action( robot_node )

    return ld

# generate_launch_description

