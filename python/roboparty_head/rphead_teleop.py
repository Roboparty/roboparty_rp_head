# SPDX-License-Identifier: GPL-3.0
"""Keyboard teleop for the two-axis head.

Keys:
  a / d  yaw left / right
  w / s  pitch up / down
  q      quit (disables the motors)

The controller's own control thread does the speed-limited interpolation; this
module only moves the target by ``step`` degrees per key press.
"""

from __future__ import annotations

import select
import sys
import time

DEFAULT_STEP = 2.0
DEFAULT_SPEED = 20.0
DEFAULT_YAW_LIMITS = (-150.0, 150.0)
DEFAULT_PITCH_LIMITS = (-20.0, 39.0)


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def _print_state(controller, yaw: float, pitch: float) -> None:
    print(
        f"target yaw={yaw:7.2f} pitch={pitch:6.2f} | "
        f"actual yaw={controller.get_joint_deg(1):7.2f} "
        f"pitch={controller.get_joint_deg(2):6.2f} | "
        f"temp={controller.get_temp(1):.0f}/{controller.get_temp(2):.0f}C "
        f"err={controller.get_error(1)}/{controller.get_error(2)}"
    )


def _demo(controller, step: float, speed: float, hold: float = 1.0) -> None:
    yaw0 = controller.get_joint_deg(1)
    pitch0 = controller.get_joint_deg(2)
    print(f"demo start yaw={yaw0:.2f} pitch={pitch0:.2f}")
    sequence = (
        (1, yaw0 + step),
        (1, yaw0),
        (2, pitch0 + step),
        (2, pitch0),
    )
    for axis, target in sequence:
        controller.move(axis, target, speed)
        time.sleep(hold)
        _print_state(
            controller,
            controller.get_joint_deg(1),
            controller.get_joint_deg(2),
        )
    print("demo done")


def _interactive(controller, yaw: float, pitch: float, step: float, speed: float) -> None:
    print("teleop: a/d yaw, w/s pitch, q quit")
    _print_state(controller, yaw, pitch)
    old_settings = None
    try:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key in ("q", "Q"):
                break
            if key in ("a", "A"):
                yaw -= step
            elif key in ("d", "D"):
                yaw += step
            elif key in ("w", "W"):
                pitch += step
            elif key in ("s", "S"):
                pitch -= step
            else:
                continue
            yaw = _clamp(yaw, *DEFAULT_YAW_LIMITS)
            pitch = _clamp(pitch, *DEFAULT_PITCH_LIMITS)
            controller.look_at(yaw, pitch, speed, speed)
            _print_state(controller, yaw, pitch)
    finally:
        if old_settings is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)


def run(
    controller,
    *,
    step: float = DEFAULT_STEP,
    speed: float = DEFAULT_SPEED,
    demo: bool = False,
    demo_hold: float = 1.0,
) -> int:
    """Enable the head and run either a scripted demo or keyboard teleop."""
    controller.set_limits(1, *DEFAULT_YAW_LIMITS)
    controller.set_limits(2, *DEFAULT_PITCH_LIMITS)
    controller.init_motors(enable=True)
    try:
        yaw = controller.get_joint_deg(1)
        pitch = controller.get_joint_deg(2)
        if demo:
            _demo(controller, step, speed, demo_hold)
        else:
            if not sys.stdin.isatty():
                print("teleop requires a terminal (use --demo for a scripted test)", file=sys.stderr)
                return 2
            _interactive(controller, yaw, pitch, step, speed)
    finally:
        controller.deinit_motors()
    return 0
