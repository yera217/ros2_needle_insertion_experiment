from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

pkg_inspt_pub = FindPackageShare( 'insertion_point_publisher' )

def generate_launch_description():
    ld = LaunchDescription()

    # arguments
    arg_ns = DeclareLaunchArgument(
            'ns',
            default_value="",
            description="ROS 2 namespace to launch the node."
            )
    arg_paramfile = DeclareLaunchArgument(
            'paramFile',
            default_value="default_params.yaml",
            description="ROS 2 launch parameter file in package share/config folder."
            )

    # nodes
    node_inspt = Node(
            package='insertion_point_publisher',
            namespace=LaunchConfiguration( 'ns' ),
            executable='insertion_point_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[ PathJoinSubstitution(
                    [ pkg_inspt_pub, 'config', LaunchConfiguration( 'paramFile' ) ]
                    ) ]
            )

    # configure launch description
    # - arguments
    ld.add_action( arg_ns )
    ld.add_action( arg_paramfile )

    # - nodes
    ld.add_action( node_inspt )

    return ld

# generate_launch_description
