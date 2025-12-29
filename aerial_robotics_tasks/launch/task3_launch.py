#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    '''
    Launch file for task3: height control, yaw control, and command multiplexing.
    Starts three nodes:
      - height_ctl_node: Controls vertical position and velocity
      - yaw_ctl_node: Controls yaw angle
      - mux_node: Multiplexes thrust and yaw_rate commands to /cf_1/cmd_vel_legacy
    '''
    
    # Height control node
    height_ctl_node = Node(
        package='aerial_robotics_tasks',
        executable='height_ctl_node',
        name='height_ctl_node',
        output='screen',
        emulate_tty=True,
    )

    # Yaw control node
    yaw_ctl_node = Node(
        package='aerial_robotics_tasks',
        executable='yaw_ctl_node',
        name='yaw_ctl_node',
        output='screen',
        emulate_tty=True,
    )

    # Mux node
    mux_node = Node(
        package='aerial_robotics_tasks',
        executable='mux_node',
        name='mux_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        height_ctl_node,
        yaw_ctl_node,
        mux_node,
    ])
