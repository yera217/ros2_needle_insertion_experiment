from email.policy import default
from pathlib import Path
from typing import Text
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

from datetime import datetime
import os

def generate_launch_description():
    ld = LaunchDescription()
    now = datetime.now().strftime( '%Y-%m-%d_%H-%M-%S' ) # Year-Month-Day_Hour-Min-Sec
    
    # arguments
    arg_fileout = DeclareLaunchArgument(
        'bagDirOut',
        default_value="bag_data/",
        description="Output bag file directory."
    )
    arg_compression = DeclareLaunchArgument(
        'useCompression',
        default_value='false',
        choices=['true', 'false'],
        description="Whether to use compression or not"
    )
    arg_maxbagsize = DeclareLaunchArgument(
        'maxBagSize',
        default_value='0',
        description="The maximum bag size (bytes) until bag is split. 0 => no splitting."
    )
    arg_topics = DeclareLaunchArgument(
        'topics',
        default_value='-a',
        description="The topics to subscribe to. Default is all topics."
    )
    
    # launch configurations
    lc_dirout = PathJoinSubstitution([LaunchConfiguration('bagDirOut'), now])
    lc_maxbagsize = LaunchConfiguration('maxBagSize')
    lc_topics = LaunchConfiguration('topics')

    # processes
    proc_bag_uncomp = ExecuteProcess(
        cmd=[ 
            'ros2', 'bag', 'record', 
            '-o', lc_dirout,
            '-b', lc_maxbagsize,
            lc_topics
        ],
        output='screen',
        condition=LaunchConfigurationEquals('useCompression', 'false'),
    )

    proc_bag_comp = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', 
            '-o', lc_dirout,
            '-b', lc_maxbagsize,
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            lc_topics
        ],
        output='screen',
        condition=LaunchConfigurationEquals('useCompression', 'true'),
    )

    # configure launch descrption
    # - arguments
    ld.add_action( arg_fileout )
    ld.add_action( arg_compression )
    ld.add_action( arg_maxbagsize )
    ld.add_action( arg_topics )

    # - processes
    ld.add_action( proc_bag_uncomp )
    ld.add_action( proc_bag_comp )

    return ld

# def