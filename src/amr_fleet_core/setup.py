from setuptools import find_packages, setup

package_name = 'amr_fleet_core'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SIH26123 Team',
    maintainer_email='team@sih26123.local',
    description=(
        'Per-robot decentralized core for SIH26123 AMR fleet coordination: '
        'space-time reservation negotiation, dual deadlock defense, CNP + RL '
        'task allocation, heartbeat fault tolerance, and explainability logging.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_agent = amr_fleet_core.fleet_agent:main',
            'goal_dispatcher = amr_fleet_core.goal_dispatcher:main',
            'collision_monitor = amr_fleet_core.collision_monitor:main',
            'initial_pose_publisher = amr_fleet_core.initial_pose_publisher:main',
            'blackboard_node = amr_fleet_core.blackboard_node:main',
            'dashboard_bridge = amr_fleet_core.dashboard_bridge:main',
            'heartbeat_node = amr_fleet_core.fault_tolerance.heartbeat_node:main',
            'task_manager_node = amr_fleet_core.task_allocation.task_manager_node:main',
            'global_planner_node = amr_fleet_core.planners.global_planner_node:main',
        ],
    },
)
