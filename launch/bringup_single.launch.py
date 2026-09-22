#!/usr/bin/env python3
"""
bringup_single.launch.py — Bring up a single robot: URDF, spawn, Nav2, initial pose.

Called once per robot by spawn_robots.launch.py with unique arguments.
Uses RewrittenYaml with root_key=robot_id so all Nav2 parameters are properly
scoped under each robot's namespace (e.g., /amr_1/controller_server).

KEY DESIGN: Everything runs under the robot's ROS2 namespace, which
automatically remaps /tf → /<ns>/tf. Plain frame names in the URDF (
base_link, odom, map) are correct for this pattern — no prefixing needed.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition

try:
    from nav2_common.launch import RewrittenYaml
except ImportError:
    import tempfile
    import yaml

    def RewrittenYaml(source_file, root_key='', param_rewrites=None, convert_types=True):
        param_rewrites = param_rewrites or {}
        with open(source_file, 'r') as f:
            data = yaml.safe_load(f) or {}

        def update_dict(d):
            for k, v in list(d.items()):
                if k in param_rewrites:
                    val = param_rewrites[k]
                    if convert_types:
                        if isinstance(val, str) and val.lower() == 'true':
                            val = True
                        elif isinstance(val, str) and val.lower() == 'false':
                            val = False
                    d[k] = val
                elif isinstance(v, dict):
                    update_dict(v)

        update_dict(data)
        if root_key:
            data = {root_key: data}

        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.safe_dump(data, tmp)
        tmp.close()
        return tmp.name


def _bringup(context, *args, **kwargs):
    """Build the actions for a single robot."""
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    src_dir = os.path.join(pkg_dir, 'src', 'amr_fleet_core')
    if src_dir not in sys.path:
        sys.path.insert(0, src_dir)

    try:
        from amr_fleet_core.scenario import ROBOT_MAP
    except ImportError:
        ROBOT_MAP = {}

    robot_id = LaunchConfiguration('robot_id').perform(context)
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context)
    chassis_color = LaunchConfiguration('chassis_color').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    urdf_path = os.path.join(pkg_dir, 'urdf', 'amr_robot.urdf.xacro')

    r_cfg = ROBOT_MAP.get(robot_id)
    if r_cfg:
        cr, cg, cb, ca = r_cfg.color_rgba
    else:
        cr, cg, cb, ca = (0.25, 0.25, 0.28, 1.0)

    # Process xacro with robot_name and RGBA color arguments
    import subprocess
    xacro_cmd = (
        f'xacro {urdf_path} '
        f'robot_name:={robot_id} '
        f'chassis_r:={cr} chassis_g:={cg} chassis_b:={cb} chassis_a:={ca}'
    )
    try:
        robot_desc = subprocess.check_output(
            xacro_cmd.split(), text=True
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        # Fallback: use raw xacro content as placeholder
        robot_desc = '<robot name="amr"><link name="base_footprint"/></robot>'

    try:
        from ament_index_python.packages import get_package_share_directory
        bt_xml_dir = os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees'
        )
        default_nav_to_pose_bt_xml = os.path.join(
            bt_xml_dir, 'navigate_to_pose_w_replanning_and_recovery.xml'
        )
        default_nav_through_poses_bt_xml = os.path.join(
            bt_xml_dir, 'navigate_through_poses_w_replanning_and_recovery.xml'
        )
    except Exception:
        default_nav_to_pose_bt_xml = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
        default_nav_through_poses_bt_xml = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml'

    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params_template.yaml')

    # Rewrite YAML parameters with robot namespace as root_key
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml,
        'default_bt_xml_filename': default_nav_to_pose_bt_xml,
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
        'map_topic': f'/{robot_id}/map',
        'set_initial_pose': 'true',
        'initial_pose.x': str(spawn_x),
        'initial_pose.y': str(spawn_y),
        'initial_pose.z': '0.0',
        'initial_pose.yaw': str(spawn_yaw),
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key=robot_id,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # Per-robot tf topic remappings to prevent multi-robot tf collision on /tf
    default_remappings = [
        ('/tf', f'/{robot_id}/tf'),
        ('/tf_static', f'/{robot_id}/tf_static'),
    ]

    # Push namespace for all child nodes
    ns_action = PushRosNamespace(robot_id)

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time == 'true',
        }],
        remappings=default_remappings,
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{robot_id}',
        output='screen',
        arguments=[
            '-entity', robot_id,
            '-topic', f'/{robot_id}/robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-Y', spawn_yaw,
            '-robot_namespace', robot_id,
        ],
    )

    # Map server (per-robot for namespace isolation)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=default_remappings,
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params, {
            'use_sim_time': use_sim_time == 'true',
            'set_initial_pose': True,
            'initial_pose.x': float(spawn_x),
            'initial_pose.y': float(spawn_y),
            'initial_pose.z': 0.0,
            'initial_pose.yaw': float(spawn_yaw),
        }],
        remappings=default_remappings,
    )

    # Controller server
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=default_remappings,
    )

    # Planner server (with map topic remapping so static_layer receives /{robot_id}/map)
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=default_remappings + [
            ('map', f'/{robot_id}/map'),
            ('/map', f'/{robot_id}/map'),
        ],
    )

    # Behavior server
    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=default_remappings,
    )

    # BT navigator
    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=default_remappings,
    )

    # Waypoint follower
    waypoint = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=default_remappings,
    )

    # Unified lifecycle manager (brings up map_server -> amcl -> navigation in order)
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[configured_params, {
            'use_sim_time': use_sim_time == 'true',
            'autostart': True,
            'attempt_respawn': True,
            'node_names': [
                'map_server', 'amcl',
                'controller_server', 'planner_server',
                'behavior_server', 'bt_navigator', 'waypoint_follower',
            ],
            'bond_timeout': 10.0,
        }],
    )

    return [GroupAction(
        actions=[
            ns_action,
            robot_state_pub,
            spawn_entity,
            map_server,
            amcl,
            controller,
            planner,
            behavior,
            bt_nav,
            waypoint,
            lifecycle_mgr,
        ]
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='amr_1'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('chassis_color', default_value='Blue'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_bringup),
    ])
