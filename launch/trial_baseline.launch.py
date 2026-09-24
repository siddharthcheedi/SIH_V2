#!/usr/bin/env python3
"""
trial_baseline.launch.py — Run the uncoordinated baseline trial.

Launches:
  1. Gazebo with warehouse.world
  2. spawn_robots.launch.py (3-robot fleet: amr_1, amr_2, amr_6)
  3. goal_dispatcher.py (sends all goals simultaneously, no negotiation)
  4. collision_monitor.py (logs ground-truth collisions/near-misses)

Output goes to benchmarks/raw/baseline_*.csv.
This is the comparison point for the 20% improvement target.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    ExecuteProcess, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    # Spawn all robots (staggered)
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

    # Set PYTHONPATH so goal_dispatcher subprocess finds amr_fleet_core reliably
    pkg_src = os.path.join(pkg_dir, 'src', 'amr_fleet_core')
    existing_pythonpath = os.environ.get('PYTHONPATH', '')
    set_pythonpath = SetEnvironmentVariable(
        'PYTHONPATH', f"{pkg_src}:{existing_pythonpath}" if existing_pythonpath else pkg_src
    )

    # Collision monitor (starts immediately)
    collision_monitor = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['python3', '-m', 'amr_fleet_core.collision_monitor',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        )],
    )

    # Goal dispatcher — waits for 3 robots to be up (~16-20s)
    # Dispatches uncoordinated NavigateToPose goals simultaneously (baseline)
    goal_dispatcher = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=['python3', '-m', 'amr_fleet_core.goal_dispatcher',
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
        collision_monitor,
        goal_dispatcher,
    ])
