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
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from lifecycle_msgs.msg import Transition

import lifecycle_msgs.srv
import rclpy
import time

def activate_lifecycle_node(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    interface = LaunchConfiguration('interface')
    auto_configure = LaunchConfiguration('auto_configure')
    auto_activate = LaunchConfiguration('auto_activate')
    timeout = LaunchConfiguration('timeout')
    transition_attempts = LaunchConfiguration('transition_attempts')

    timeout_s = float(timeout.perform(context))

    rclpy.init()
    node = rclpy.create_node(f'{interface.perform(context)}_socket_can_sender_activator')

    cli = node.create_client(
        lifecycle_msgs.srv.ChangeState,
        f'{namespace.perform(context)}/{interface.perform(context)}_socket_can_sender/change_state')
    if not cli.wait_for_service(timeout_sec=timeout_s):
        node.get_logger().error('Lifecycle service not available.')
        return

    retry_count = int(transition_attempts.perform(context))
    req = lifecycle_msgs.srv.ChangeState.Request()

    if auto_configure.perform(context) == 'true':
      req.transition.id = Transition.TRANSITION_CONFIGURE
      for i in range(retry_count):
          future = cli.call_async(req)
          rclpy.spin_until_future_complete(node, future)
          if future.result() and future.result().success:
              node.get_logger().info('Lifecycle node configured successfully.')
              break
          else:
              node.get_logger().warn(f'Activation attempt {i+1} failed. Retrying...')
              time.sleep(timeout_s)

    if auto_activate.perform(context) == 'true':
      req.transition.id = Transition.TRANSITION_ACTIVATE
      for i in range(retry_count):
          future = cli.call_async(req)
          rclpy.spin_until_future_complete(node, future)
          if future.result() and future.result().success:
              node.get_logger().info('Lifecycle node activated successfully.')
              break
          else:
              node.get_logger().warn(f'Activation attempt {i+1} failed. Retrying...')
              time.sleep(timeout_s)

    node.destroy_node()
    rclpy.shutdown()

    return []

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    interface = LaunchConfiguration('interface')
    enable_can_fd = LaunchConfiguration('enable_can_fd')
    timeout_sec = LaunchConfiguration('timeout_sec')
    to_can_bus_topic = LaunchConfiguration('to_can_bus_topic')

    node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name=f'{interface.perform(context)}_socket_can_sender',
        namespace=namespace,
        parameters=[{
            'interface': interface.perform(context),
            'enable_can_fd': enable_can_fd.perform(context) == 'true',
            'timeout_sec': float(timeout_sec.perform(context)),
        }],
        remappings=[('to_can_bus', to_can_bus_topic.perform(context))],
        output='screen')

    # Wait for interface to be up
    wait_for_can_interface_proc = ExecuteProcess(
        cmd=[['until ', FindExecutable(name='ip'), ' link show ', interface.perform(context),
              ' | ', FindExecutable(name='grep'), ' \"state UP\"', '; do sleep 1; done']],
        shell=True
    )

    launch_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_can_interface_proc,
            on_exit=node
        )
    )

    configure_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node,
            on_start=[
              OpaqueFunction(function=activate_lifecycle_node)
            ],
        ),
    )

    return [
        wait_for_can_interface_proc,
        launch_node,
        configure_event,
    ]


def generate_launch_description():
    arg_namespace = DeclareLaunchArgument(
      'namespace',
      default_value='')

    arg_interface = DeclareLaunchArgument(
      'interface',
      default_value='can0')

    arg_enable_can_fd = DeclareLaunchArgument(
      'enable_can_fd',
      default_value='false')

    arg_timeout_sec = DeclareLaunchArgument(
      'timeout_sec',
      default_value='1.0')

    arg_auto_configure = DeclareLaunchArgument(
      'auto_configure',
      default_value='true')

    arg_auto_activate = DeclareLaunchArgument(
      'auto_activate',
      default_value='true')

    arg_to_can_bus_topic = DeclareLaunchArgument(
      'to_can_bus_topic',
      default_value='tx')

    arg_timeout = DeclareLaunchArgument(
      'timeout',
      default_value='5.0')

    arg_transition_attempts = DeclareLaunchArgument(
      'transition_attempts',
      default_value='3')

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_interface)
    ld.add_action(arg_enable_can_fd)
    ld.add_action(arg_timeout_sec)
    ld.add_action(arg_auto_configure)
    ld.add_action(arg_auto_activate)
    ld.add_action(arg_to_can_bus_topic)
    ld.add_action(arg_timeout)
    ld.add_action(arg_transition_attempts)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
