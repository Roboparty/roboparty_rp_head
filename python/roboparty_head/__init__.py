# SPDX-License-Identifier: GPL-3.0
"""Pure-Python two-axis head controller built on ``motors_py``."""

__version__ = "0.2.1"

from .rphead_controller import (
    AXIS_PITCH,
    AXIS_YAW,
    DEFAULT_CONTROL_DT,
    DEFAULT_KD,
    DEFAULT_KP,
    DEFAULT_LIMIT_MAX,
    DEFAULT_LIMIT_MIN,
    DEFAULT_SPEED_DEG_S,
    HeadController,
    step_towards,
)

__all__ = [
    "__version__",
    "AXIS_PITCH",
    "AXIS_YAW",
    "DEFAULT_CONTROL_DT",
    "DEFAULT_KD",
    "DEFAULT_KP",
    "DEFAULT_LIMIT_MAX",
    "DEFAULT_LIMIT_MIN",
    "DEFAULT_SPEED_DEG_S",
    "HeadController",
    "step_towards",
]
