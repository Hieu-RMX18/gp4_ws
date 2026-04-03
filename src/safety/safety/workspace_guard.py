from geometry_msgs.msg import Pose

class WorkspaceGuard:
    def __init__(self, safety_rules: dict):
        self.bounds = safety_rules.get("workspace_bounds", {})
        self.forbidden_zones = safety_rules.get("forbidden_zones", [])

    def check_pose(self, pose: Pose) -> tuple[bool, str]:
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        # Check workspace bounds
        x_min = self.bounds.get("x_min", -10.0)
        x_max = self.bounds.get("x_max", 10.0)
        y_min = self.bounds.get("y_min", -10.0)
        y_max = self.bounds.get("y_max", 10.0)
        z_min = self.bounds.get("z_min", -10.0)
        z_max = self.bounds.get("z_max", 10.0)

        if not (x_min <= x <= x_max):
            return False, f"X coordinate {x} out of bounds [{x_min}, {x_max}]"
        if not (y_min <= y <= y_max):
            return False, f"Y coordinate {y} out of bounds [{y_min}, {y_max}]"
        if not (z_min <= z <= z_max):
            return False, f"Z coordinate {z} out of bounds [{z_min}, {z_max}]"

        # Check forbidden zones (AABB: x,y,z is center point)
        for zone in self.forbidden_zones:
            name = zone.get("name", "unknown")
            zx = zone.get("x", 0.0)
            zy = zone.get("y", 0.0)
            zz = zone.get("z", 0.0)
            sx = zone.get("size_x", 0.0)
            sy = zone.get("size_y", 0.0)
            sz = zone.get("size_z", 0.0)

            min_x = zx - sx / 2.0
            max_x = zx + sx / 2.0
            min_y = zy - sy / 2.0
            max_y = zy + sy / 2.0
            min_z = zz - sz / 2.0
            max_z = zz + sz / 2.0

            if (min_x <= x <= max_x) and (min_y <= y <= max_y) and (min_z <= z <= max_z):
                return False, f"collision with {name}"

        return True, ""
