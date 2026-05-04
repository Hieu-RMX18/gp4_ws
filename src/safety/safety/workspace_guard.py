"""Workspace bound & forbidden-zone geometric validation.

Boundary & contract
═══════════════════
:class:`WorkspaceGuard` is a **pure geometric checker**. It does *not* query
TF, transform frames, read robot state, or subscribe to any topic. Callers
must supply a pose **already expressed in the robot ``base_link`` frame**
and in **SI units (meters)** — the same frame and units used by
``safety_rules.yaml`` for ``workspace_bounds`` and ``forbidden_zones``.

Frame convention
────────────────
- **Accepted frame:** ``base_link`` (robot base).
- **Accepted units:** meters (SI). Callers holding millimeters / degrees
  must normalize upstream (see ``llm_gateway.normalizer``).
- **Orientation:** ignored. This guard checks translation only.
- **End-effector flange vs TCP:** the caller decides which point is
  checked. Station bounds in ``safety_rules.yaml`` are defined for the
  point MoveIt plans for, typically the TCP frame attached to
  ``base_link`` via TF. Passing a raw flange pose without tool offset
  will under-report collisions near the table.

Responsibilities
────────────────
- Returns ``(False, reason)`` when the pose violates axis bounds or
  enters any forbidden AABB.
- Returns ``(True, "")`` otherwise.
- **Does NOT** validate: reachability, singularity, joint limits, orientation,
  velocity/acceleration, approval state, motion primitive type, MOVE_REL
  deltas, CIRC auxiliary waypoints. Those live in ``ExecutionGate``,
  ``CommandValidator``, ``move_rel_validator.hpp`` (C++ side), MoveIt,
  or upstream validators.

Failure mode
────────────
Fail-closed for AABB geometry only. A missing/empty ``workspace_bounds`` in
safety_rules falls back to permissive ±10m defaults — callers relying on
fail-closed semantics MUST load a populated ``safety_rules.yaml`` via
``safety.policy_loader.load_safety_rules()``.

Defense-in-depth siblings
─────────────────────────
1. ``ExecutionGate`` — request-level validation, MOVE_REL delta check.
2. ``move_rel_validator.hpp`` (C++) — re-validates resolved MOVE_REL
   target against the same bounds inside ``motion_core``.
3. ``MoveIt`` — collision-aware planning with ``station_mesh``
   (authoritative geometry).
"""

from geometry_msgs.msg import Pose


class WorkspaceGuard:
    """Geometric AABB validator for a single Cartesian point in ``base_link``.

    Parameters
    ----------
    safety_rules : dict
        Parsed ``safety_rules.yaml``. Expected top-level keys:

        - ``workspace_bounds``: dict with ``x_min/x_max/y_min/y_max/z_min/z_max``
          (meters, ``base_link`` frame). Missing axes fall back to ±10 m.
        - ``forbidden_zones``: list of AABB dicts with ``x, y, z`` (center,
          meters) and ``size_x, size_y, size_z`` (full extents, meters),
          plus optional ``name`` for the reject reason string.

    See the module docstring for the full contract.
    """

    def __init__(self, safety_rules: dict):
        self.bounds = safety_rules.get("workspace_bounds", {})
        self.forbidden_zones = safety_rules.get("forbidden_zones", [])

    def check_pose(self, pose: Pose) -> tuple[bool, str]:
        """Validate a single pose against workspace bounds and forbidden zones.

        Parameters
        ----------
        pose : geometry_msgs.msg.Pose
            **Must be expressed in ``base_link`` and meters.** Orientation
            is ignored; only ``position.{x,y,z}`` is inspected.

        Returns
        -------
        (valid, reason) : tuple[bool, str]
            ``(True, "")`` when the pose is inside bounds and outside every
            forbidden zone. ``(False, reason)`` otherwise, where ``reason``
            names the first failing axis or zone.

        Notes
        -----
        - The forbidden-zone containment test uses closed inequalities on
          all three axes (``min ≤ coord ≤ max``). A pose touching a zone
          boundary counts as a collision.
        - The workspace-bound test is also closed on both ends.
        """
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

            if (
                (min_x <= x <= max_x)
                and (min_y <= y <= max_y)
                and (min_z <= z <= max_z)
            ):
                return False, f"collision with {name}"

        return True, ""
