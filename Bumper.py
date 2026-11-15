
"""Local collision avoidance policy for a VEX AI computer.

Implement RobotIO with the VEX AI vision and V5-brain communication APIs used by
your robot. Unknown vision clearance is treated as blocked.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Protocol, Tuple


MIN_CLEARANCE_MM = 500.0
FORWARD_SPEED = 0.20
TURN_SPEED = 0.25
POLL_INTERVAL_SECONDS = 0.02


@dataclass(frozen=True)
class SensorSnapshot:
    """Clearances are conservative; None means the direction is unobserved."""

    front_pressed: bool
    rear_pressed: bool
    left_pressed: bool
    right_pressed: bool
    front_clearance_mm: Optional[float]
    rear_clearance_mm: Optional[float]
    left_clearance_mm: Optional[float]
    right_clearance_mm: Optional[float]


class RobotIO(Protocol):
    """Hardware adapter for sensors and V5-brain drive commands."""

    def read_snapshot(self) -> SensorSnapshot:
        """Read switches and calibrated AI Vision clearances.

        Return None for any direction the sensor cannot currently verify as clear.
        """
        ...

    def drive(self, forward: float, turn: float) -> None:
        """Send normalized forward and turn commands to the drivetrain."""
        ...

    def stop(self) -> None:
        """Stop the drivetrain immediately."""
        ...


class AvoidanceController:
    def __init__(
        self,
        io: RobotIO,
        min_clearance_mm: float = MIN_CLEARANCE_MM,
        forward_speed: float = FORWARD_SPEED,
        turn_speed: float = TURN_SPEED,
    ) -> None:
        if not math.isfinite(min_clearance_mm) or min_clearance_mm <= 0:
            raise ValueError("min_clearance_mm must be a positive finite value")
        if not 0 < forward_speed <= 1:
            raise ValueError("forward_speed must be between 0 and 1")
        if not 0 < turn_speed <= 1:
            raise ValueError("turn_speed must be between 0 and 1")

        self._io = io
        self._min_clearance_mm = min_clearance_mm
        self._forward_speed = forward_speed
        self._turn_speed = turn_speed

    def command_for(self, snapshot: SensorSnapshot) -> Optional[Tuple[float, float]]:
        """Return (forward, turn), or None when no path is known to be safe."""
        contacts = {
            "front": snapshot.front_pressed,
            "rear": snapshot.rear_pressed,
            "left": snapshot.left_pressed,
            "right": snapshot.right_pressed,
        }
        clearances = {
            "front": snapshot.front_clearance_mm,
            "rear": snapshot.rear_clearance_mm,
            "left": snapshot.left_clearance_mm,
            "right": snapshot.right_clearance_mm,
        }

        def path_is_clear(direction: str) -> bool:
            clearance = clearances[direction]
            return (
                clearance is not None
                and math.isfinite(clearance)
                and clearance >= self._min_clearance_mm
            )

        if any(contacts.values()):
            opposite = {
                "front": "rear",
                "rear": "front",
                "left": "right",
                "right": "left",
            }
            candidates = [opposite[side] for side, pressed in contacts.items() if pressed]
            candidates.extend(("left", "right", "rear", "front"))

            for direction in candidates:
                if contacts[direction] or not path_is_clear(direction):
                    continue
                if direction == "front":
                    return self._forward_speed, 0.0
                if direction == "rear":
                    return -self._forward_speed, 0.0
                if direction == "left":
                    return 0.0, -self._turn_speed
                return 0.0, self._turn_speed

            return None

        if path_is_clear("front"):
            return self._forward_speed, 0.0

        turn_options = (
            ("left", -self._turn_speed),
            ("right", self._turn_speed),
        )
        clear_turns = [
            (clearances[direction], turn)
            for direction, turn in turn_options
            if path_is_clear(direction)
        ]
        if clear_turns:
            _, turn = max(clear_turns, key=lambda option: option[0])
            return 0.0, turn

        return None

    def run(self) -> None:
        """Run until interrupted; always stop the drivetrain on exit."""
        try:
            while True:
                command = self.command_for(self._io.read_snapshot())
                if command is None:
                    self._io.stop()
                else:
                    self._io.drive(*command)
                time.sleep(POLL_INTERVAL_SECONDS)
        finally:
            self._io.stop()
