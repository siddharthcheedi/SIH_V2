"""
task_allocation — Contract Net Protocol + RL bid-priority layer.

The RL policy's ONLY output is a bid-priority scalar. It has no access to
cmd_vel, reservation data, or any motion interface. This is a structural
guarantee enforced by the module's public API, not a convention.
"""
