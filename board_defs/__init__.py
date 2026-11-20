"""机械臂型号参数。通过 get_profile()、arm.board_defs 或 link.board_defs 获取。"""

from . import arx_x5, metal_arm

_PROFILES = {"arx_x5": arx_x5, "metal_arm": metal_arm}
ROBOT_TYPES = tuple(_PROFILES)


def get_profile(robot_type):
    """返回型号参数；未知型号抛 ValueError。"""
    try:
        return _PROFILES[robot_type]
    except (KeyError, TypeError):
        raise ValueError("unknown robot_type %r; choose from %s"
                         % (robot_type, ", ".join(ROBOT_TYPES))) from None


__all__ = ["arx_x5", "metal_arm", "ROBOT_TYPES", "get_profile"]
