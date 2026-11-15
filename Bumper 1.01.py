# this is just a add on to the previous bumper code to detect any mistakes that the previous version may have as i want this to be the best version there is

import math
from optparse import Option
from os import read
import time
from dataclasses import dataclass
from turtle import right
from typing import Optional, Protocol
from vex import * 

MIN_CLEARANCE_MM= 500.00
# this indicates that if the bumper were to be pressed within 0.5 meters of itself it would be 
FORWARD_SPEED= 0.25
#indicates the speed of the forward movement after the bumper were to be pressed
TURN_SPEED= 0.25
#indicates turn speed if the robot were to be pressed within the bumper, as it could hit an object such as a wall, another robot, or pins
POLL_INTERVAL_SECONDS= 0.10
#indicates the interval between each poll to check the bumper status

@dataclass(frozen=True)
class SensorSnapshot:
    front_pressed: bool
    rear_pressed: bool
    left_pressed: bool
    right_pressed: bool
    front_clearance_mm: Optional[float] = None
    rear_clearance_mm: Optional[float] = None
    left_clearance_mm: Optional[float] = None
    right_clearance_mm: Optional[float] = None
# this is a comment to demonstrate what shryan has done rq 
# front, rear, left, and right each respectfully store whether the bumper is pressed or not becuz bool is true/false
# each clearance mm for each bumper stores a distance measurement in millimeters or none if the direction has no given reading
#@dataclass frozen true wtv makes these values an immutable snapshot once made

class RobotIO(Protocol):
    def read_snapshot(self) -> SensorSnapshot:
        ...

    def drive(self, forward: float, turn: float) -> None:
        ...

    def stop(self) -> None:
        ...
    #ok so read_snapshot reads the current bumper state and returns a SensorSnapshot
    #drive moves the robot with the given forward and turn values
    #stop stops the robot 
    #tuff ahh code lwk

# this is defining a new object for connecting to our vex hardware so tuff ik it is currently 11:24 pm rn i need sleep
class VexAIRobotIO:
    #vexairobot io is a container for the references to the four bumper devices that we have along wiht the one distance sensor i provide when creating the robot
    def __init__(
        self,
        front_bumper,
        rear_bumper,
        left_bumper,
        right_bumper,
        distance_sensor,
        left_motor,
        right_motor,
    ) -> None:
        self._front_bumper = front_bumper
        self._rear_bumper = rear_bumper
        self._left_bumper = left_bumper
        self._right_bumper = right_bumper
        self._distance_sensor = distance_sensor
        self._left_motor = left_motor
        self._right_motor = right_motor

    def read_snapshot(self) -> SensorSnapshot:
        # read the current distance from the distance sensor in millimeters
        # takes one reading from the distance sensor and checks whether each bumper is pressed
        distance_mm = self._distance_sensor.object_distance(MM)

        return SensorSnapshot(
            front_pressed=self._front_bumper.pressing(),
            rear_pressed=self._rear_bumper.pressing(),
            left_pressed=self._left_bumper.pressing(),
            right_pressed=self._right_bumper.pressing(),
            front_clearance_mm=distance_mm,
            rear_clearance_mm=None,
            left_clearance_mm=None,
            right_clearance_mm=None,
        )

    def drive(self, forward: float, turn: float) -> None:
        left_speed = max(-1.0, min(1.0, forward + turn))
        right_speed = max(-1.0, min(1.0, forward - turn))
        self._spin_motor(self._left_motor, left_speed)
        self._spin_motor(self._right_motor, right_speed)

    def _spin_motor(self, motor, speed: float) -> None:
        direction = FORWARD if speed >= 0 else REVERSE
        motor.spin(direction, abs(speed) * 100, PERCENT)

    def stop(self) -> None:
        self._left_motor.stop()
        self._right_motor.stop()


class BumperAvoidanceController:
    def __init__(self, robot: RobotIO) -> None:
        self._robot = robot
# so what this basicallly does is it saves the provided robot interfaace as a self._robot, so the controller can use it later. 
    def step(self) -> None:
        # we are defining step and this code runs when another part of the program calls that method
        snapshot = self._robot.read_snapshot()
        #calls read snapnshot to get the current bumper states and distance reading, then saves them in snanpshot 
        front_blocked = snapshot.front_pressed or (
            snapshot.front_clearance_mm is not None
            and snapshot.front_clearance_mm <= MIN_CLEARANCE_MM
            #sets front blocked to ture if the front bumper is pressed, otherwise it checks whether a front distance reading exists and is 500 mm or less. if neither conditon i strue, front blocked is false instead
        )

        if (
            (snapshot.front_pressed and snapshot.rear_pressed)
            or (snapshot.left_pressed and snapshot.right_pressed)
        ):
            # Stop if bumpers on opposite sides are pressed; otherwise, handle a blocked front.
            self._robot.stop()
        elif front_blocked:
            if snapshot.rear_pressed:
                self._robot.stop()
            else:
                # Assumes positive turn means right, negative means left.
                turn = -TURN_SPEED if snapshot.right_pressed else TURN_SPEED
                self._robot.drive(-FORWARD_SPEED, turn)
        elif snapshot.rear_pressed:
            self._robot.drive(FORWARD_SPEED, 0.0)
        elif snapshot.left_pressed:
            self._robot.drive(FORWARD_SPEED, TURN_SPEED)
        elif snapshot.right_pressed:
            self._robot.drive(FORWARD_SPEED, -TURN_SPEED)
        else:
            self._robot.drive(FORWARD_SPEED, 0.0)


def main() -> None:
    # Replace these placeholders with the devices configured for your VEX robot.
    front_bumper = None
    rear_bumper = None
    left_bumper = None
    right_bumper = None
    distance_sensor = None
    left_motor = None
    right_motor = None

    devices = (
        front_bumper,
        rear_bumper,
        left_bumper,
        right_bumper,
        distance_sensor,
        left_motor,
        right_motor,
    )
    if any(device is None for device in devices):
        raise RuntimeError(
            "Configure all bumper, distance sensor, and drivetrain device "
            "placeholders before running."
        )

    robot = VexAIRobotIO(
        front_bumper,
        rear_bumper,
        left_bumper,
        right_bumper,
        distance_sensor,
        left_motor,
        right_motor,
    )
    controller = BumperAvoidanceController(robot)

    while True:
        controller.step()
        wait(POLL_INTERVAL_SECONDS, SECONDS)


if __name__ == "__main__":
    main()
