from dataclasses import dataclass
from typing import Optional, Protocol
from vex import *

MIN_CLEARANCE_MM = 500.0
FORWARD_SPEED = 0.25
TURN_SPEED = 0.25
POLL_INTERVAL_SECONDS = 0.02  # Faster 20ms loop for quicker collision response

@dataclass(frozen=True)
class SensorSnapshot:
    front_pressed: bool
    rear_pressed: bool
    left_pressed: bool
    right_pressed: bool
    front_clearance_mm: Optional[float] = None

class RobotIO(Protocol):
    def read_snapshot(self) -> SensorSnapshot: ...
    def drive(self, forward: float, turn: float) -> None: ...
    def stop(self) -> None: ...

class VexAIRobotIO:
    def __init__(
        self,
        front_bumper: Bumper,
        rear_bumper: Bumper,
        left_bumper: Bumper,
        right_bumper: Bumper,
        distance_sensor: Distance,
        left_motors: MotorGroup,
        right_motors: MotorGroup,
    ) -> None:
        self._front_bumper = front_bumper
        self._rear_bumper = rear_bumper
        self._left_bumper = left_bumper
        self._right_bumper = right_bumper
        self._distance_sensor = distance_sensor
        self._left_motors = left_motors
        self._right_motors = right_motors

    def read_snapshot(self) -> SensorSnapshot:
        # Only trust the distance reading if an object is actually detected
        distance_mm = None
        if self._distance_sensor.is_object_detected():
            distance_mm = self._distance_sensor.object_distance(MM)

        return SensorSnapshot(
            front_pressed=self._front_bumper.pressing(),
            rear_pressed=self._rear_bumper.pressing(),
            left_pressed=self._left_bumper.pressing(),
            right_pressed=self._right_bumper.pressing(),
            front_clearance_mm=distance_mm,
        )

    def drive(self, forward: float, turn: float) -> None:
        left_speed = max(-1.0, min(1.0, forward + turn))
        right_speed = max(-1.0, min(1.0, forward - turn))
        self._spin_group(self._left_motors, left_speed)
        self._spin_group(self._right_motors, right_speed)

    def _spin_group(self, group: MotorGroup, speed: float) -> None:
        direction = FORWARD if speed >= 0 else REVERSE
        group.spin(direction, abs(speed) * 100, PERCENT)

    def stop(self) -> None:
        self._left_motors.stop()
        self._right_motors.stop()

class BumperAvoidanceController:
    def __init__(self, robot: RobotIO) -> None:
        self._robot = robot

    def step(self) -> bool:
        """Returns True if avoidance took over the drivetrain, False if clear."""
        snapshot = self._robot.read_snapshot()
        
        front_blocked = snapshot.front_pressed or (
            snapshot.front_clearance_mm is not None
            and snapshot.front_clearance_mm <= MIN_CLEARANCE_MM
        )

        # Trapped on opposite sides
        if (snapshot.front_pressed and snapshot.rear_pressed) or (
            snapshot.left_pressed and snapshot.right_pressed
        ):
            self._robot.stop()
            return True

        # Front blocked: back up and turn away from side obstacles
        if front_blocked:
            if snapshot.rear_pressed:
                self._robot.stop()
            else:
                turn = -TURN_SPEED if snapshot.right_pressed else TURN_SPEED
                self._robot.drive(-FORWARD_SPEED, turn)
            return True

        # Rear hit: drive forward
        if snapshot.rear_pressed:
            self._robot.drive(FORWARD_SPEED, 0.0)
            return True

        # Side hits: drive forward and steer away
        if snapshot.left_pressed:
            self._robot.drive(FORWARD_SPEED, TURN_SPEED)
            return True

        if snapshot.right_pressed:
            self._robot.drive(FORWARD_SPEED, -TURN_SPEED)
            return True

        # Path is clear: do NOT override main drivetrain commands!
        return False
