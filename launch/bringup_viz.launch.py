# Copyright (c) 2024，D-Robotics.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    namespace = LaunchConfiguration('namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='True',
        description='Whether to use composition')
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='False',
        description='Whether to use re-spawning of processes')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', 
        default_value='tros_container',
        description='the name of the container that launches all nodes')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', 
        default_value='warn',
        description='log level')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            exec_name='rectified_image_jpeg_encoder_node',
            parameters=[
                {'in_mode': 'ros'},
                {'out_mode': 'ros'},
                {'in_format': 'nv12'},
                {'out_format': 'jpeg'},
                {'pub_topic': '/image_jpeg'},
                {'sub_topic': '/StereoNetNode/rectified_image'}
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'
        ),
    ])
    
    apritag_det = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tros_apriltag_det'),
                'launch/bringup.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': container_name,
        }.items()
    )
    
    # web viz
    # ros2 launch websocket websocket.launch.py \
    # websocket_image_topic:=/image_jpeg websocket_only_show_image:=false websocket_smart_topic:=/tros_refined_tag_position
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg',
            'websocket_only_show_image': 'false',
            'websocket_smart_topic': '/tros_refined_tag_position',
            'log_level': log_level
        }.items()
    )

    # tros_lowpass_filter_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('tros_lowpass_filter'),
    #                      'launch/low_pass.launch.py')),
    #     launch_arguments={
    #         'lowpass_perc_sub_topic': 'tros_apriltag_detections',
    #         'lowpass_perc_pub_topic': 'tros_apriltag_detections_filtered',
    #         'log_level': log_level
    #     }.items()
    # )

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(bringup_cmd_group)
    ld.add_action(apritag_det)
    # ld.add_action(tros_lowpass_filter_node)
    ld.add_action(web_node)

    return ld
