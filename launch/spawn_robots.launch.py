#!/usr/bin/env python3
"""
spawn_robots.launch.py — Staggered multi-robot spawner.

Spawns all robots defined in scenario.py into a running Gazebo instance
with 2-second stagger between each to avoid entity-creation race conditions.

Uses IncludeLaunchDescription to invoke bringup_single.launch.py for each
robot, which handles URDF→robot_description, spawn_entity, Nav2 bringup,
and initial-pose seeding.

This file reads from scenario.ROBOTS — all robot poses, colors, and
namespaces are defined there. If you change the fleet, edit scenario.py,
not this file.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, IncludeLaunchDescription,
    GroupAction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Import fleet config — available because amr_fleet_core is installed
import sys
# Add the package source to sys.path for launch-time import
_PKG_DIR = os.path.join(os.path.dirname(__file__), '..', 'src',
                         'amr_fleet_core')
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from amr_fleet_core.scenario import ROBOTS


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Force FastDDS to use pure UDPv4 to avoid shared-memory (SHM) port lock exhaustion
    fastdds_profile = os.path.join(pkg_dir, 'config', 'fastdds_udp.xml')
    set_fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile
    )
    set_fastdds_transports = SetEnvironmentVariable(
        'FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'warehouse.world'),
        description='Path to the Gazebo world file',
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'warehouse_map.yaml'),
        description='Path to the Nav2 map YAML',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo',
    )

    # Build staggered spawn actions — 4.5s apart to avoid CPU/DDS contention
    spawn_actions = []
    for i, robot in enumerate(ROBOTS):
        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'bringup_single.launch.py')
            ),
            launch_arguments={
                'robot_id': robot.robot_id,
                'spawn_x': str(robot.spawn_x),
                'spawn_y': str(robot.spawn_y),
                'spawn_yaw': str(robot.spawn_yaw),
                'chassis_color': robot.color_name,
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        )

        if i == 0:
            spawn_actions.append(bringup)
        else:
            spawn_actions.append(
                TimerAction(period=float(i * 4.5), actions=[bringup])
            )

    return LaunchDescription([
        set_fastdds_profile,
        set_fastdds_transports,
        world_arg,
        map_arg,
        use_sim_time_arg,
        *spawn_actions,
    ])
