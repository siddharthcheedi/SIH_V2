"""
negotiation — Pure-Python, zero-ROS2-dependency negotiation core.

Every module in this package is deliberately kept free of any rclpy, ROS2
message, or ROS2 service import so that the entire conflict-resolution and
deadlock-detection pipeline can be tested with plain ``pytest`` on any machine
— including one with no ROS2 installation at all.

ROS2 nodes are thin wrappers around the logic in this package.
"""
