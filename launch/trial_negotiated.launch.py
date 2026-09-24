#!/usr/bin/env python3
"""
trial_negotiated.launch.py — Run the negotiated (coordinated) trial.

Launches:
  1. Gazebo with warehouse.world
  2. spawn_robots.launch.py (3-robot fleet: amr_1, amr_2, amr_6)
  3. Per-robot fleet_agent nodes (negotiation + ORCA + heartbeat)
  4. Blackboard node (information only)
  5. collision_monitor.py (logs ground-truth collisions/near-misses)
  6. dashboard_bridge.py (observer-only telemetry)

Output goes to benchmarks/raw/negotiated_*.csv.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    ExecuteProcess, GroupAction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Import fleet config
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

    world_path = os.path.join(pkg_dir, 'worlds', 'warehouse.world')
    map_path = os.path.join(pkg_dir, 'maps', 'warehouse_map.yaml')

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_init.so', world_path],
        output='screen',
    )

    # Spawn all robots
    spawn_robots = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'spawn_robots.launch.py')
            ),
            launch_arguments={
                'map': map_path,
                'use_sim_time': 'true',
            }.items(),
        )],
    )

    # Set PYTHONPATH so fleet_agent subprocesses find amr_fleet_core reliably
    pkg_src = os.path.join(pkg_dir, 'src', 'amr_fleet_core')
    existing_pythonpath = os.environ.get('PYTHONPATH', '')
    set_pythonpath = SetEnvironmentVariable(
        'PYTHONPATH', f"{pkg_src}:{existing_pythonpath}" if existing_pythonpath else pkg_src
    )

    # Per-robot fleet agent nodes — launched once all 3 robots are active (~16s)
    # Fleet: amr_1 (Red), amr_2 (Blue), amr_6 (Cyan)
    fleet_agents = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '-m', 'amr_fleet_core.fleet_agent',
                     robot.robot_id, '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen',
            )
            for robot in ROBOTS
        ],
    )

    # Blackboard (information only)
    blackboard = TimerAction(
        period=18.0,
        actions=[ExecuteProcess(
            cmd=['python3', '-m', 'amr_fleet_core.blackboard_node',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        )],
    )

    # Collision monitor
    collision_monitor = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['python3', '-m', 'amr_fleet_core.collision_monitor',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        )],
    )

    # Dashboard bridge
    dashboard_bridge = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            cmd=['python3', '-m', 'amr_fleet_core.dashboard_bridge',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        )],
    )

    return LaunchDescription([
        set_fastdds_profile,
        set_fastdds_transports,
        set_pythonpath,
        gazebo,
        spawn_robots,
        fleet_agents,
        blackboard,
        collision_monitor,
        dashboard_bridge,
    ])
