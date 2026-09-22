"""
amr_fleet_core — Per-robot decentralized core for SIH26123 AMR fleet coordination.

Every robot runs an identical copy of this package. No robot is special or a
hidden leader. Collision avoidance, deadlock resolution, and task allocation
all happen via peer-to-peer broadcast — the shared blackboard holds map/task
state only, never a reservation or a collision-avoidance decision.
"""
