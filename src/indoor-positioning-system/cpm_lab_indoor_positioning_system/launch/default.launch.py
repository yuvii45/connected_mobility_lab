# Copyright 2025 Cyber-Physical Mobility Group
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Define default config file path
    default_config_file: str = os.path.join(
        get_package_share_directory('cpm_lab_indoor_positioning_system'),
        'config',
        'default.yaml'
    )

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        description='IPS Parameters and Calibration structured in a .yaml file.',
        default_value=default_config_file
    )

    # Configurations
    config_file = LaunchConfiguration('config_file')

    # Construct namespace as "/lab/"
    namespace = '/lab/'

    # Define the composable node
    node = ComposableNode(
        package='cpm_lab_indoor_positioning_system',
        plugin='indoor_positioning_system_ros::VehicleDetectionNode',
        name='indoor_positioning_system',
        namespace=namespace,
        parameters=[
            config_file,
        ]
    )

    container = ComposableNodeContainer(
        name='indoor_positioning_system_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[node],
    )

    return LaunchDescription([
        config_file_arg,
        container
    ])
