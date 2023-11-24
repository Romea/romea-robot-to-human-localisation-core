# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node


def get_filter_name(context):
    return LaunchConfiguration("filter_name").perform(context)


def get_filter_configuration(context):
    with open(LaunchConfiguration("filter_configuration_file_path").perform(context)) as f:
        return yaml.safe_load(f)


def get_component_container(context):
    return LaunchConfiguration("component_container").perform(context)


def launch_setup(context, *args, **kwargs):
    filter_name = get_filter_name(context)
    filter_configuration = get_filter_configuration(context)

    component_container = get_component_container(context)
    if not component_container:

        node = Node(
            package="romea_robot_to_human_localisation_core",
            executable="robot_to_human_localisation_node",
            name=filter_name,
            parameters=[filter_configuration],
            output="screen",
        )

        return [node]
    else:
        composable_node = ComposableNode(
                package="romea_robot_to_human_localisation_core",
                plugin="romea::ros2::R2HLocalisation",
                name=filter_name,
                parameters=[filter_configuration],
            )

        load_component = LoadComposableNodes(
            composable_node_descriptions=[composable_node],
            target_container=component_container),

        return [load_component]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("filter_name", default_value="robot_to_human_localisation"))

    declared_arguments.append(DeclareLaunchArgument("filter_configuration_file_path"))

    declared_arguments.append(DeclareLaunchArgument("component_container", default_value=""))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
