# Copyright 2022 Andrew Symington
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def _launch_setup(context):
    config_dir = LaunchConfiguration('config_dir').perform(context).strip()
    force_recalibrate = LaunchConfiguration('force_recalibrate').perform(context).strip()
    tracking_frame = LaunchConfiguration('tracking_frame').perform(context).strip()
    world_frame = LaunchConfiguration('world_frame').perform(context).strip()

    driver_args = ""
    if force_recalibrate == "true":
        driver_args = "--force-recalibrate 1"
        if config_dir:
            driver_args += f" -c {config_dir}/libsurvive/config.json"

    parameters = [
        {'driver_args': driver_args},
        {'tracking_frame': tracking_frame},
        {'imu_topic': 'imu'},
        {'joy_topic': 'joy'},
        {'cfg_topic': 'cfg'},
        {'velocity_topic': 'velocity'},
        {'battery_topic': 'battery'},
        {'occlusion_topic': 'occlusion'},
        {'lighthouse_rate': 4.0}
    ]

    extra_env = {'XDG_CONFIG_HOME': config_dir} if config_dir else {}

    # Non-composable launch (regular node)
    libsurvive_node = Node(
        package='libsurvive_ros2',
        executable='libsurvive_ros2_node',
        name='libsurvive_ros2_node',
        namespace=LaunchConfiguration('namespace'),
        condition=UnlessCondition(LaunchConfiguration('composable')),
        output='screen',
        additional_env=extra_env,
        parameters=parameters)

    # Composable launch (zero-copy node example)
    libsurvive_composable_node = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='libsurvive_ros2_container',
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('composable')),
        additional_env=extra_env,
        composable_node_descriptions=[
            ComposableNode(
                package='libsurvive_ros2',
                plugin='libsurvive_ros2::Component',
                name='libsurvive_ros2_component',
                parameters=parameters,
                extra_arguments=[
                    {'use_intra_process_comms': True}
                ]
            )
        ],
        output='log')

    # For recording all data from the experiment
    world_align_node = Node(
        package='libsurvive_ros2',
        executable='libsurvive_world_align_node',
        name='libsurvive_world_align_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'tracking_frame': tracking_frame},
            {'world_frame': world_frame},
            {'joy_topic': 'joy'},
        ])

    return [
        libsurvive_node,
        libsurvive_composable_node,
        world_align_node,
    ]


def generate_launch_description():
    default_config_dir = os.path.join(get_package_share_directory('libsurvive_ros2'), 'config')

    arguments = [
        DeclareLaunchArgument('namespace', default_value='libsurvive',
                              description='Namespace for the non-TF topics'),
        DeclareLaunchArgument('composable', default_value='false',
                              description='Launch in a composable container'),
        DeclareLaunchArgument('tracking_frame', default_value='libsurvive_world',
                              description='Frame used as the parent frame for tracked poses'),
        DeclareLaunchArgument('world_frame', default_value='world',
                              description='World frame name for static alignment transform'),
        DeclareLaunchArgument('force_recalibrate', default_value='false',
                              description='Whether to force a fresh libsurvive calibration'),
        DeclareLaunchArgument('config_dir', default_value=default_config_dir,
                              description=('Path to a libsurvive calibration config directory. '
                                           f'Default: {default_config_dir}')),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
