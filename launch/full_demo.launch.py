#!/usr/bin/env python3
"""
full_demo.launch.py — Complete demo: negotiated fleet + dashboard.

Combines trial_negotiated.launch.py with the FastAPI dashboard server.
Used for the live SIH demonstration.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    launch_dir = os.path.join(pkg_dir, 'launch')
    dashboard_dir = os.path.join(pkg_dir, 'dashboard')

    # Launch the negotiated trial
    negotiated_trial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'trial_negotiated.launch.py')
        ),
    )

    # Launch dashboard server (after everything else is up)
    dashboard = TimerAction(
        period=30.0,
        actions=[ExecuteProcess(
            cmd=[sys.executable, '-m', 'uvicorn',
                 'app:app', '--host', '0.0.0.0', '--port', '8080'],
            cwd=dashboard_dir,
            output='screen',
        )],
    )

    return LaunchDescription([
        negotiated_trial,
        dashboard,
    ])
