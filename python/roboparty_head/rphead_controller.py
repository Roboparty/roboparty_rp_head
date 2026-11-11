# SPDX-License-Identifier: GPL-3.0
"""Pure-Python two-axis head controller built on ``motors_py``.

Mirrors ``src/head_controller.cpp`` (LRO_PJ2_55_5550, MIT PD servo at 100 Hz) without
building the C++ library. By default ``init_motors(enable=False)`` only creates the
drivers and reads feedback; enabling and motion require ``enable=True`` explicitly.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Iterable

try:
    import motors_py  # type: ignore
except ImportError:  # pragma: no cover - depends on the board install
    motors_py = None

AXIS_YAW = 1
AXIS_PITCH = 2
_AXIS_ALIASES = {
    "yaw": AXIS_YAW,
    "pitch": AXIS_PITCH,
    "1": AXIS_YAW,
    "2": AXIS_PITCH,
}

LRO_MODEL_5550 = 0
DEFAULT_LIMIT_MIN = (-120.0, -20.0)  # [yaw, pitch] deg
DEFAULT_LIMIT_MAX = (120.0, 39.0)
DEFAULT_KP = 100.0
DEFAULT_KD = 2.0
DEFAULT_CONTROL_DT = 0.01
DEFAULT_SPEED_DEG_S = 30.0


def _axis_index(axis: int | str) -> int:
    if isinstance(axis, int):
        if axis == AXIS_YAW:
            return 0
        if axis == AXIS_PITCH:
            return 1
        raise ValueError("axis must be 1/yaw or 2/pitch")
    key = str(axis).strip().lower()
    if key in _AXIS_ALIASES:
        return 0 if _AXIS_ALIASES[key] == AXIS_YAW else 1
    raise ValueError("axis must be 1/yaw or 2/pitch")


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def step_towards(current: float, target: float, max_step: float) -> float:
    """Move ``current`` toward ``target`` by at most ``max_step``."""
    diff = target - current
    if abs(diff) <= max_step:
        return target
    return current + (max_step if diff > 0 else -max_step)


class HeadController:
    """Two LRO head motors (axis 1 = yaw, axis 2 = pitch) driven by ``motors_py``."""

    def __init__(
        self,
        can_interface: str = "can0",
        yaw_id: int = 1,
        pitch_id: int = 2,
        *,
        interface_type: str = "can",
        kp: float = DEFAULT_KP,
        kd: float = DEFAULT_KD,
        control_dt: float = DEFAULT_CONTROL_DT,
        motor_factory: Callable[..., Any] | None = None,
    ) -> None:
        if motor_factory is None:
            if motors_py is None:
                raise RuntimeError("motors_py is not installed")
            motor_factory = motors_py.MotorDriver.create_motor
        self._factory = motor_factory
        self.can_interface = can_interface
        self.interface_type = str(interface_type)
        self.yaw_id = int(yaw_id)
        self.pitch_id = int(pitch_id)
        self._ids = (self.yaw_id, self.pitch_id)
        self._kp = (float(kp), float(kp))
        self._kd = (float(kd), float(kd))
        self._dt = float(control_dt)

        self._limit_min = list(DEFAULT_LIMIT_MIN)
        self._limit_max = list(DEFAULT_LIMIT_MAX)
        self._motors: list[Any | None] = [None, None]

        self._lock = threading.RLock()
        self._final = [0.0, 0.0]   # rad
        self._current = [0.0, 0.0]  # rad
        self._speed = [DEFAULT_SPEED_DEG_S, DEFAULT_SPEED_DEG_S]  # deg/s
        self._enabled = False
        self._opened = False
        self._thread: threading.Thread | None = None
        self._running = False
        self._refresh_index = 0  # alternate refresh target: yaw, pitch, yaw, ...

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def probe(self, timeout: float = 0.2) -> list[dict[str, Any]]:
        """Read-only: create drivers, request feedback, return status. Never enables."""
        try:
            self._create_motors()
            self._wait_feedback(timeout)
            return self.status()
        finally:
            # Drop the drivers without sending enable/disable; the C++ destructor
            # removes the CAN callback.
            self._release_motors()

    def init_motors(self, enable: bool = False, feedback_timeout: float = 0.1) -> None:
        """Create motors. With ``enable=False`` only feedback is read.

        With ``enable=True`` this calls the driver's ``init_motor()`` (which enables the
        motors), switches to MIT mode and starts the 100 Hz control thread.
        """
        self._create_motors()
        if not enable:
            self._wait_feedback(feedback_timeout)
            return

        for index, motor in enumerate(self._motors):
            assert motor is not None
            error = int(motor.init_motor())
            if error != 0:
                self._release_motors()
                raise RuntimeError(f"head motor init failed (id={self._ids[index]}, error={error})")
            if motors_py is not None:
                motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)

        self._wait_feedback(feedback_timeout)
        with self._lock:
            for i, motor in enumerate(self._motors):
                assert motor is not None
                self._current[i] = float(motor.get_motor_pos())
                self._final[i] = self._current[i]
            self._opened = True
            self._enabled = True
        self._start_control()

    def enable(self, feedback_timeout: float = 0.1) -> None:
        """Enable both motors and resume the control thread."""
        if not self._motors[0] or not self._motors[1]:
            raise RuntimeError("head motors not initialized: call init_motors() first")
        for motor in self._motors:
            assert motor is not None
            if motors_py is not None:
                motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
            motor.lock_motor()
        try:
            self._wait_feedback(feedback_timeout)
            with self._lock:
                for i, motor in enumerate(self._motors):
                    assert motor is not None
                    self._current[i] = float(motor.get_motor_pos())
                    self._final[i] = self._current[i]
                self._opened = True
                self._enabled = True
            self._start_control()
        except Exception:
            for motor in self._motors:
                assert motor is not None
                motor.unlock_motor()
            raise

    def disable(self) -> None:
        """Stop commanding and disable both motors."""
        self._stop_control()
        with self._lock:
            self._enabled = False
        for motor in self._motors:
            if motor is not None:
                motor.unlock_motor()

    def deinit_motors(self) -> None:
        """Stop, disable and release both motor drivers."""
        self.disable()
        self._release_motors()
        with self._lock:
            self._opened = False

    def close(self) -> None:
        self.deinit_motors()

    def __enter__(self) -> "HeadController":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.deinit_motors()

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def move(self, axis: int | str, deg: float, speed: float = DEFAULT_SPEED_DEG_S) -> float:
        """Set one axis target (deg). Non-blocking; returns the clamped target."""
        index = _axis_index(axis)
        target = _clamp(float(deg), self._limit_min[index], self._limit_max[index])
        with self._lock:
            self._require_enabled()
            self._final[index] = math.radians(target)
            self._speed[index] = speed if speed > 0 else DEFAULT_SPEED_DEG_S
        return target

    def look_at(
        self,
        yaw_deg: float,
        pitch_deg: float,
        yaw_speed: float = DEFAULT_SPEED_DEG_S,
        pitch_speed: float = DEFAULT_SPEED_DEG_S,
    ) -> tuple[float, float]:
        """Set yaw/pitch targets together (deg). Non-blocking; returns clamped targets."""
        yaw = _clamp(float(yaw_deg), self._limit_min[0], self._limit_max[0])
        pitch = _clamp(float(pitch_deg), self._limit_min[1], self._limit_max[1])
        with self._lock:
            self._require_enabled()
            self._final[0] = math.radians(yaw)
            self._final[1] = math.radians(pitch)
            self._speed[0] = yaw_speed if yaw_speed > 0 else DEFAULT_SPEED_DEG_S
            self._speed[1] = pitch_speed if pitch_speed > 0 else DEFAULT_SPEED_DEG_S
        return yaw, pitch

    # ------------------------------------------------------------------
    # Queries / configuration
    # ------------------------------------------------------------------

    def status(self) -> list[dict[str, Any]]:
        result = []
        for index, motor in enumerate(self._motors):
            if motor is None:
                result.append({"axis": index + 1, "present": False})
                continue
            result.append({
                "axis": index + 1,
                "name": "yaw" if index == 0 else "pitch",
                "id": self._ids[index],
                "present": True,
                "deg": float(motor.get_motor_pos()) * 180.0 / math.pi,
                "temp_c": float(motor.get_motor_temperature()),
                "error": int(motor.get_error_id()),
                "pending": int(motor.get_response_count()),
            })
        return result

    def get_joint_deg(self, axis: int | str) -> float:
        motor = self._motor(axis)
        return float(motor.get_motor_pos()) * 180.0 / math.pi

    def get_temp(self, axis: int | str) -> float:
        return float(self._motor(axis).get_motor_temperature())

    def get_error(self, axis: int | str) -> int:
        return int(self._motor(axis).get_error_id())

    def set_limits(self, axis: int | str, min_deg: float, max_deg: float) -> None:
        if min_deg > max_deg:
            raise ValueError("min_deg must be <= max_deg")
        index = _axis_index(axis)
        self._limit_min[index] = float(min_deg)
        self._limit_max[index] = float(max_deg)

    def clear_error(self, axis: int | str, *, confirm: bool = False) -> None:
        """Write operation. Requires ``confirm=True`` (user authorization)."""
        if not confirm:
            raise PermissionError("clear_error requires confirm=True (explicit authorization)")
        self._motor(axis).clear_motor_error()

    def set_zero(self, axis: int | str, *, confirm: bool = False) -> None:
        """Write operation that changes the motor zero. Requires ``confirm=True``."""
        if not confirm:
            raise PermissionError("set_zero requires confirm=True (explicit authorization)")
        if self._enabled:
            raise RuntimeError("set_zero requires disabled motors; call disable() first")
        self._motor(axis).set_motor_zero()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _require_enabled(self) -> None:
        if not self._enabled:
            raise RuntimeError("head motors are not enabled: call init_motors(enable=True) or enable()")

    def _motor(self, axis: int | str) -> Any:
        index = _axis_index(axis)
        motor = self._motors[index]
        if motor is None:
            raise RuntimeError("head motors not initialized: call init_motors() first")
        return motor

    def _create_motors(self) -> None:
        if self._motors[0] is not None and self._motors[1] is not None:
            return
        self._release_motors()
        created: list[Any] = []
        try:
            for motor_id in self._ids:
                created.append(self._factory(
                    motor_id=motor_id,
                    interface_type=self.interface_type,
                    interface=self.can_interface,
                    motor_type="LRO",
                    motor_model=LRO_MODEL_5550,
                    master_id_offset=0,
                    motor_zero_offset=0.0,
                ))
        except Exception:
            created.clear()
            raise
        self._motors = [created[0], created[1]]

    def _release_motors(self) -> None:
        self._motors = [None, None]

    def _wait_feedback(self, timeout: float) -> None:
        motors = [motor for motor in self._motors if motor is not None]
        if not motors:
            return
        # LRO's first refresh only switches the local control mode; set MIT
        # explicitly so the query frame is actually sent.
        if motors_py is not None:
            for motor in motors:
                motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
        for motor in motors:
            motor.refresh_motor_status()
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if all(int(motor.get_response_count()) == 0 for motor in motors):
                return
            time.sleep(0.001)
        if any(int(motor.get_response_count()) != 0 for motor in motors):
            self._release_motors()
            raise RuntimeError("head motor feedback timeout")

    def _start_control(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, name="head-control", daemon=True)
        self._thread.start()

    def _stop_control(self) -> None:
        self._running = False
        thread = self._thread
        self._thread = None
        if thread and thread.is_alive():
            thread.join(timeout=1.0)

    def _control_loop(self) -> None:
        next_release = time.monotonic()
        refresh_tick = 0
        while self._running:
            next_release += self._dt
            commands: list[tuple[Any, float, float, float]] = []
            with self._lock:
                if self._enabled:
                    for i, motor in enumerate(self._motors):
                        if motor is None:
                            continue
                        step = math.radians(self._speed[i]) * self._dt
                        self._current[i] = step_towards(self._current[i], self._final[i], step)
                        commands.append((motor, self._current[i], self._kp[i], self._kd[i]))
            for motor, position, kp, kd in commands:
                motor.motor_mit_cmd(position, 0.0, kp, kd, 0.0)
            if commands:
                refresh_tick += 1
                if refresh_tick >= 25:
                    refresh_tick = 0
                    motor = self._motors[self._refresh_index]
                    self._refresh_index ^= 1
                    if motor is not None:
                        # Guard against colliding with the regular MIT frames and
                        # the motor's feedback; back-to-back refresh frames can
                        # push classic CAN TEC into ERROR-PASSIVE.
                        time.sleep(0.001)
                        motor.refresh_motor_status()

            now = time.monotonic()
            if now > next_release:
                next_release = now
            time.sleep(max(0.0, next_release - now))
