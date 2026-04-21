# DEPRECATED (2026-04-21): Not referenced by gp4_bringup entrypoints.
# Use gp4_bringup/launch/moveit_only.launch.py for supported operator flow.
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("motoman_gp4", package_name="gp4_moveit_config").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
