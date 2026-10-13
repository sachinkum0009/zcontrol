# Copyright 2026 Sachin Kumar
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Fake rover (ESP32-S3 or virtual device) driven by diff_drive_controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    endpoint = LaunchConfiguration("endpoint")
    device = LaunchConfiguration("device")

    pkg = FindPackageShare("zc_bringup")
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution([pkg, "urdf", "fake_rover.urdf.xacro"]),
                " endpoint:=",
                endpoint,
                " device:=",
                device,
            ]
        ),
        value_type=str,
    )
    controllers = PathJoinSubstitution([pkg, "config", "fake_rover_controllers.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "endpoint",
                default_value="tcp/127.0.0.1:7447",
                description="zenoh router the plugin connects to",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="rover1",
                description="device name, keys are zc/<device>/state and zc/<device>/cmd",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "diff_drive_controller",
                    "--param-file",
                    controllers,
                ],
            ),
        ]
    )
