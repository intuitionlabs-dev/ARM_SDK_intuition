"""Per-robot SDK definitions.

Use ``from sdk.board_defs import arx_x5`` or ``metal_arm``, or obtain the
selected module from ``arm.board_defs`` / ``link.board_defs``. This package
intentionally exposes no default joint limits or reset pose.
"""

from . import arx_x5, metal_arm

__all__ = ["arx_x5", "metal_arm"]
