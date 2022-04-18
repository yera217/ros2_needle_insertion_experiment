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
    arg_point_x = DeclareLaunchArgument(
            'x',
            default_value="0.0",
            description="Insertion: publish point x-coordinate"
            )
    arg_point_y = DeclareLaunchArgument(
            'y',
            default_value="0.0",
            description="Insertion: publish point y-coordinate"
            )
    arg_point_z = DeclareLaunchArgument(
            'z',
            default_value="0.0",
            description="Insertion: publish point z-coordinate"
            )

    arg_pub_time = DeclareLaunchArgument(
            'publishTime',
            default_value="0.1",
            description="Time (seconds) in how often the insertion point will publish"
            )

    # nodes
    node_inspt = Node(
            package='insertion_point_publisher',
            namespace=LaunchConfiguration( 'ns' ),
            executable='insertion_point_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[ {
                    'insertion.point.x'           : LaunchConfiguration( 'x' ),
                    'insertion.point.y'           : LaunchConfiguration( 'y' ),
                    'insertion.point.z'           : LaunchConfiguration( 'z' ),
                    'insertion.point.publish.time': LaunchConfiguration( 'publishTime' ),
                    } ]
            )

    # configure launch description
    # - arguments
    ld.add_action( arg_ns )
    ld.add_action( arg_point_x )
    ld.add_action( arg_point_y )
    ld.add_action( arg_point_z )
    ld.add_action( arg_pub_time )

    # - nodes
    ld.add_action( node_inspt )

    return ld

# generate_launch_description
