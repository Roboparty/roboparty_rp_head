# SPDX-License-Identifier: GPL-3.0
"""Offline tests for the motors_py head controller (no CAN hardware touched)."""

from __future__ import annotations

import pathlib
import sys
import threading
import time
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "python"))

from roboparty_head.controller import (  # noqa: E402
    AXIS_PITCH,
    AXIS_YAW,
    HeadController,
    step_towards,
)


class FakeMotor:
    def __init__(self, motor_id: int) -> None:
        self.motor_id = motor_id
        self.calls: list[object] = []
        self.position = 0.25
        self.temperature = 31.0
        self.error = 0
        self.mit_event = threading.Event()

    def init_motor(self) -> int:
        self.calls.append("init_motor")
        return 0

    def lock_motor(self) -> None:
        self.calls.append("lock_motor")

    def unlock_motor(self) -> None:
        self.calls.append("unlock_motor")

    def set_motor_control_mode(self, mode: object) -> None:
        self.calls.append("set_mode")

    def refresh_motor_status(self) -> None:
        self.calls.append("refresh")

    def get_response_count(self) -> int:
        return 0

    def get_motor_pos(self) -> float:
        return self.position

    def get_motor_temperature(self) -> float:
        return self.temperature

    def get_error_id(self) -> int:
        return self.error

    def motor_mit_cmd(self, position: float, velocity: float, kp: float, kd: float,
                      torque: float) -> None:
        self.position = position
        self.calls.append("mit")
        self.mit_event.set()

    def clear_motor_error(self) -> None:
        self.calls.append("clear_error")

    def set_motor_zero(self) -> bool:
        self.calls.append("set_zero")
        return True


class FakeFactory:
    def __init__(self) -> None:
        self.motors: dict[int, FakeMotor] = {}

    def __call__(self, **kwargs: object) -> FakeMotor:
        motor = FakeMotor(int(kwargs["motor_id"]))  # type: ignore[arg-type]
        self.motors[motor.motor_id] = motor
        return motor


class HeadControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.factory = FakeFactory()
        self.controller = HeadController(can_interface="vcan-test", yaw_id=1, pitch_id=2,
                                         motor_factory=self.factory)

    def tearDown(self) -> None:
        self.controller.deinit_motors()

    def test_probe_is_read_only(self) -> None:
        status = self.controller.probe()

        self.assertEqual([item["axis"] for item in status], [AXIS_YAW, AXIS_PITCH])
        for motor in self.factory.motors.values():
            self.assertNotIn("init_motor", motor.calls)
            self.assertNotIn("lock_motor", motor.calls)
            self.assertNotIn("mit", motor.calls)
            self.assertIn("refresh", motor.calls)

    def test_init_without_enable_does_not_touch_torque(self) -> None:
        self.controller.init_motors(enable=False)

        for motor in self.factory.motors.values():
            self.assertNotIn("init_motor", motor.calls)
            self.assertNotIn("lock_motor", motor.calls)
            self.assertNotIn("mit", motor.calls)

    def test_init_with_enable_starts_control_thread(self) -> None:
        self.controller.init_motors(enable=True)
        try:
            from roboparty_head import controller as module

            motor = self.factory.motors[1]
            self.assertIn("init_motor", motor.calls)
            if module.motors_py is not None:
                self.assertIn("set_mode", motor.calls)
            self.assertTrue(motor.mit_event.wait(1.0), "control thread did not send MIT")
        finally:
            self.controller.disable()

        self.assertIn("unlock_motor", motor.calls)
        self.assertFalse(self.controller._enabled)  # noqa: SLF001 - test probe

    def test_move_requires_enable_and_clamps(self) -> None:
        self.controller.init_motors(enable=False)
        with self.assertRaises(RuntimeError):
            self.controller.move("yaw", 10)

        self.controller.init_motors(enable=True)
        try:
            self.assertEqual(self.controller.move("yaw", 999), 120.0)
            self.assertEqual(self.controller.move("pitch", -999), -20.0)
            yaw, pitch = self.controller.look_at(999, 999)
            self.assertEqual((yaw, pitch), (120.0, 39.0))
        finally:
            self.controller.disable()

    def test_axis_validation(self) -> None:
        with self.assertRaises(ValueError):
            self.controller.move(3, 0)

    def test_write_operations_require_confirmation(self) -> None:
        self.controller.init_motors(enable=False)
        with self.assertRaises(PermissionError):
            self.controller.clear_error("yaw")
        with self.assertRaises(PermissionError):
            self.controller.set_zero("yaw")

    def test_fake_factory_without_motors_py(self) -> None:
        from roboparty_head import controller as module

        previous = module.motors_py
        module.motors_py = None
        controller = HeadController(can_interface="vcan-test", motor_factory=self.factory)
        try:
            controller.init_motors(enable=True)
            motor = self.factory.motors[1]
            self.assertTrue(motor.mit_event.wait(1.0))
            controller.disable()
        finally:
            controller.deinit_motors()
            module.motors_py = previous

    def test_step_towards(self) -> None:
        self.assertAlmostEqual(step_towards(0.0, 1.0, 0.3), 0.3)
        self.assertAlmostEqual(step_towards(0.9, 1.0, 0.3), 1.0)
        self.assertAlmostEqual(step_towards(0.0, -1.0, 0.3), -0.3)


if __name__ == "__main__":
    unittest.main()
