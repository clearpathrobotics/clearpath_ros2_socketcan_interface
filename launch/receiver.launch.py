# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition



def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    interface = LaunchConfiguration('interface')
    enable_can_fd = LaunchConfiguration('enable_can_fd')
    interval_sec = LaunchConfiguration('interval_sec')
    use_bus_time = LaunchConfiguration('use_bus_time')
    filters = LaunchConfiguration('filters')
    auto_configure = LaunchConfiguration('auto_configure')
    auto_activate = LaunchConfiguration('auto_activate')
    from_can_bus_topic = LaunchConfiguration('from_can_bus_topic')

    arg_namespace = DeclareLaunchArgument(
      'namespace',
      default_value='')

    arg_interface = DeclareLaunchArgument(
      'interface',
      default_value='can0')

    arg_enable_can_fd = DeclareLaunchArgument(
      'enable_can_fd',
      default_value='false')

    arg_interval_sec = DeclareLaunchArgument(
      'interval_sec',
      default_value='0.01')

    arg_use_bus_time = DeclareLaunchArgument(
        'use_bus_time',
        default_value='false')

    arg_filters = DeclareLaunchArgument(
        'filters',
        default_value='0:0')

    arg_auto_configure = DeclareLaunchArgument(
      'auto_configure',
      default_value='true')

    arg_auto_activate = DeclareLaunchArgument(
      'auto_activate',
      default_value='true')

    arg_from_can_bus_topic = DeclareLaunchArgument(
      'from_can_bus_topic',
      default_value='rx')

    node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver',
        namespace=namespace,
        parameters=[{
            'interface': interface,
            'enable_can_fd': enable_can_fd,
            'interval_sec': interval_sec,
            'filters': filters,
            'use_bus_time': use_bus_time,
        }],
        remappings=[('from_can_bus', from_can_bus_topic)],
        output='screen')

    configure_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(auto_configure),
    )

    activate_event = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(auto_activate),
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_interface)
    ld.add_action(arg_enable_can_fd)
    ld.add_action(arg_interval_sec)
    ld.add_action(arg_use_bus_time)
    ld.add_action(arg_filters)
    ld.add_action(arg_auto_configure)
    ld.add_action(arg_auto_activate)
    ld.add_action(arg_from_can_bus_topic)
    ld.add_action(node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    return ld
