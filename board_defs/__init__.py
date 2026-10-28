"""Per-robot SDK definitions.

Use ``from sdk.board_defs import arx_x5`` or ``metal_arm``, or obtain the
selected module from ``arm.board_defs`` / ``link.board_defs``. This package
intentionally exposes no default joint limits or reset pose.
"""

from . import arx_x5, metal_arm

_PROFILES = {"arx_x5": arx_x5, "metal_arm": metal_arm}
ROBOT_TYPES = tuple(_PROFILES)


def get_profile(robot_type):
    """Resolve an explicit robot type; reject unknown names before opening hardware."""
    try:
        return _PROFILES[robot_type]
    except (KeyError, TypeError):
        raise ValueError("unknown robot_type %r; choose from %s"
                         % (robot_type, ", ".join(ROBOT_TYPES))) from None


__all__ = ["arx_x5", "metal_arm", "ROBOT_TYPES", "get_profile"]
