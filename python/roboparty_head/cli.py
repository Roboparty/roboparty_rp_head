#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
"""Command-line / interactive control for the two-axis LRO head via motors_py.

Safety:
  * ``probe`` / ``status`` are read-only.
  * ``init``, ``enable``, ``move`` and ``look`` enable torque and require ``--yes``
    (or an explicit interactive confirmation).
  * The pitch axis droops under gravity when disabled.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from .controller import (
    DEFAULT_CONTROL_DT,
    DEFAULT_KD,
    DEFAULT_KP,
    DEFAULT_SPEED_DEG_S,
    HeadController,
)


def _dump(value: object) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2))


def _confirm(args: argparse.Namespace, action: str) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print(f"拒绝执行 {action}: 非交互环境必须显式加 --yes", file=sys.stderr)
        return False
    answer = input(f"即将执行 {action}（会使能电机）。输入 yes 继续: ").strip().lower()
    return answer == "yes"


def _controller(args: argparse.Namespace) -> HeadController:
    return HeadController(
        can_interface=args.interface,
        yaw_id=args.yaw_id,
        pitch_id=args.pitch_id,
        kp=args.kp,
        kd=args.kd,
        control_dt=args.dt,
    )


def _print_banner() -> None:
    print("RoboParty head control (motors_py)")
    print("安全前提：机器人可靠支撑、急停可达、电源限流、总线 ERROR-ACTIVE。")
    print("probe/status 只读；init/enable/move/look 会使能电机。")
    print("pitch 轴在 disable 后会因重力下坠。\n")


def _run_once(args: argparse.Namespace) -> int:
    command = args.command
    if command == "probe":
        _dump(_controller(args).probe())
        return 0

    if command == "status":
        _dump(_controller(args).probe())
        return 0

    if command == "limits":
        controller = _controller(args)
        controller.set_limits(args.axis, args.min, args.max)
        print(f"axis {args.axis}: [{args.min}, {args.max}] deg")
        return 0

    if command == "init":
        if args.enable and not _confirm(args, "init --enable"):
            return 2
        controller = _controller(args)
        try:
            controller.init_motors(enable=args.enable)
            _dump(controller.status())
            if args.enable and args.hold > 0:
                time.sleep(args.hold)
        finally:
            controller.deinit_motors()
        return 0

    if command == "enable":
        if not _confirm(args, "enable"):
            return 2
        controller = _controller(args)
        try:
            controller.init_motors(enable=False)
            controller.enable()
            _dump(controller.status())
            if args.hold > 0:
                time.sleep(args.hold)
        finally:
            controller.deinit_motors()
        return 0

    if command == "disable":
        controller = _controller(args)
        try:
            controller.disable()
            print("disabled (pitch axis droops under gravity)")
        finally:
            controller.deinit_motors()
        return 0

    if command in ("move", "look"):
        if not _confirm(args, command):
            return 2
        controller = _controller(args)
        try:
            controller.init_motors(enable=True)
            if command == "move":
                target = controller.move(args.axis, args.deg, args.speed)
                print(f"move axis={args.axis} target={target:.2f} deg speed={args.speed} deg/s")
            else:
                yaw, pitch = controller.look_at(args.yaw, args.pitch, args.yaw_speed, args.pitch_speed)
                print(f"look yaw={yaw:.2f} pitch={pitch:.2f} deg")
            if args.hold > 0:
                time.sleep(args.hold)
            _dump(controller.status())
        finally:
            controller.deinit_motors()
        return 0

    print(f"unknown command: {command}", file=sys.stderr)
    return 2


def _interactive(args: argparse.Namespace) -> int:
    _print_banner()
    controller = _controller(args)
    try:
        while True:
            try:
                line = input("head> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            parts = line.split()
            command = parts[0].lower()
            try:
                if command in ("quit", "exit", "q"):
                    break
                if command in ("probe", "status"):
                    _dump(controller.probe())
                elif command == "init":
                    if not _confirm(args, "init"):
                        continue
                    controller.init_motors(enable=True)
                    _dump(controller.status())
                elif command == "enable":
                    if not _confirm(args, "enable"):
                        continue
                    controller.enable()
                    _dump(controller.status())
                elif command == "disable":
                    controller.disable()
                    print("disabled (pitch axis droops under gravity)")
                elif command == "move":
                    if len(parts) < 3:
                        print("usage: move <yaw|pitch|1|2> <deg> [speed]")
                        continue
                    if not _confirm(args, "move"):
                        continue
                    speed = float(parts[3]) if len(parts) > 3 else DEFAULT_SPEED_DEG_S
                    target = controller.move(parts[1], float(parts[2]), speed)
                    print(f"target={target:.2f} deg")
                elif command == "look":
                    if len(parts) < 3:
                        print("usage: look <yaw_deg> <pitch_deg> [yaw_speed] [pitch_speed]")
                        continue
                    if not _confirm(args, "look"):
                        continue
                    yaw_speed = float(parts[3]) if len(parts) > 3 else DEFAULT_SPEED_DEG_S
                    pitch_speed = float(parts[4]) if len(parts) > 4 else DEFAULT_SPEED_DEG_S
                    yaw, pitch = controller.look_at(float(parts[1]), float(parts[2]),
                                                    yaw_speed, pitch_speed)
                    print(f"yaw={yaw:.2f} pitch={pitch:.2f} deg")
                elif command == "limits":
                    if len(parts) != 4:
                        print("usage: limits <axis> <min_deg> <max_deg>")
                        continue
                    controller.set_limits(parts[1], float(parts[2]), float(parts[3]))
                    print("limits updated")
                else:
                    print("commands: probe/status/init/enable/disable/move/look/limits/quit")
            except Exception as exc:  # keep the REPL alive
                print(f"error: {exc}", file=sys.stderr)
    finally:
        controller.deinit_motors()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface", default="can0", help="CAN-FD interface (default: can0)")
    parser.add_argument("--yaw-id", type=int, default=1, help="yaw motor CAN ID (default: 1)")
    parser.add_argument("--pitch-id", type=int, default=2, help="pitch motor CAN ID (default: 2)")
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--dt", type=float, default=DEFAULT_CONTROL_DT)
    parser.add_argument("--yes", action="store_true",
                        help="explicitly confirm enabling motors / motion")

    sub = parser.add_subparsers(dest="command")
    sub.add_parser("probe", help="read-only feedback")
    sub.add_parser("status", help="read-only feedback (alias of probe)")

    init = sub.add_parser("init", help="create drivers; --enable also enables motors")
    init.add_argument("--enable", action="store_true")
    init.add_argument("--hold", type=float, default=1.0)

    enable = sub.add_parser("enable", help="enable motors")
    enable.add_argument("--hold", type=float, default=1.0)

    sub.add_parser("disable", help="disable motors")

    move = sub.add_parser("move", help="move one axis")
    move.add_argument("axis")
    move.add_argument("deg", type=float)
    move.add_argument("--speed", type=float, default=DEFAULT_SPEED_DEG_S)
    move.add_argument("--hold", type=float, default=1.0)

    look = sub.add_parser("look", help="move both axes")
    look.add_argument("--yaw", type=float, default=0.0)
    look.add_argument("--pitch", type=float, default=0.0)
    look.add_argument("--yaw-speed", type=float, default=DEFAULT_SPEED_DEG_S)
    look.add_argument("--pitch-speed", type=float, default=DEFAULT_SPEED_DEG_S)
    look.add_argument("--hold", type=float, default=1.0)

    limits = sub.add_parser("limits", help="set software limits (no hardware write)")
    limits.add_argument("axis")
    limits.add_argument("min", type=float)
    limits.add_argument("max", type=float)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command is None:
        return _interactive(args)
    return _run_once(args)


if __name__ == "__main__":
    sys.exit(main())
