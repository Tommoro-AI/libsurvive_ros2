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

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _clear_existing_config(config_dir):
    config_root = Path(config_dir) / 'libsurvive'
    config_root.mkdir(parents=True, exist_ok=True)
    config_file = config_root / 'config.json'
    config_file.write_text('', encoding='utf-8')


def _launch_setup(context):
    config_dir = LaunchConfiguration('config_dir').perform(context).strip()
    force_recalibrate = LaunchConfiguration(
        'force_recalibrate').perform(context).strip()
    if config_dir:
        _clear_existing_config(config_dir)

    driver_args = ''
    if force_recalibrate == 'true':
        driver_args = '--force-recalibrate 1'
        if config_dir:
            driver_args += f' -c {config_dir}/libsurvive/config.json'

    parameters = [
        {'driver_args': driver_args},
        {'imu_topic': 'imu'},
        {'joy_topic': 'joy'},
        {'cfg_topic': 'cfg'},
        {'velocity_topic': 'velocity'},
        {'battery_topic': 'battery'},
        {'confidence_topic': 'confidence'},
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
        output='screen',
        additional_env=extra_env,
        parameters=parameters)

    return [
        libsurvive_node,
    ]


def generate_launch_description():
    default_config_dir = PathJoinSubstitution([
        FindPackageShare('libsurvive_ros2'),
        'config'
    ])

    arguments = [
        DeclareLaunchArgument('namespace', default_value='libsurvive',
                              description='Namespace for the non-TF topics'),
        DeclareLaunchArgument('force_recalibrate', default_value='false',
                              description='Whether to force a fresh libsurvive calibration'),
        DeclareLaunchArgument('config_dir', default_value=default_config_dir,
                              description=('Path to a libsurvive calibration config directory. '
                                           f'Default: {default_config_dir}')),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
