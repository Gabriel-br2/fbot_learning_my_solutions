#!/usr/bin/env python3
"""
Turtle trajectory challenge using YASMIN state machine.

DEV     : Gabriel
PROJECT : ROS2 Turtlesim Challenge

USAGE:
    ros2 run turtlesim turtlesim_node
    ros2 run fbot_c3_pkg turtle_control_c3_node 
"""

import math
from dataclasses import dataclass
from enum import Enum, auto

import rclpy
import yasmin
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from yasmin import Blackboard, State, StateMachine
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED
from yasmin_ros.yasmin_node import YasminNode


class StepAction(Enum):
    MOVE   = auto()
    ROTATE = auto()


@dataclass(frozen=True)
class MissionStep:
    name   : str
    action : StepAction
    value  : float
    speed  : float = 1.0


class TurtleState(State):
    """
    Base class for all turtle motion states.

    Handles the shared ROS2 infrastructure: publisher creation,
    pose subscription, and the initial pose wait loop. Subclasses
    only need to implement the motion logic via `_run()`.

    Note: ABC cannot be used here due to a metaclass conflict with
    yasmin.State. The abstract contract is enforced via NotImplementedError.
    """

    def __init__(self) -> None:
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL])

        self._node           = YasminNode.get_instance()
        self._vel_publisher  = self._node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._pose_sub       = self._node.create_subscription(
            Pose, '/turtle1/pose', self._pose_callback, 10
        )
        self._pose: Pose | None = None

    def _pose_callback(self, msg: Pose) -> None:
        self._pose = msg

    def _wait_for_pose(self) -> bool:
        """Spins until the first pose message arrives. Returns False if ROS shuts down."""
        while self._pose is None and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)
        return rclpy.ok()

    def execute(self, blackboard: Blackboard) -> str:
        if not self._wait_for_pose():
            return ABORT
        return self._run()

    def _run(self) -> str:
        """Executes the motion logic. Called once pose is guaranteed to be available."""
        raise NotImplementedError(f"{type(self).__name__} must implement _run()")


class MoveState(TurtleState):
    def __init__(self, distance: float, speed: float = 1.0) -> None:
        super().__init__()
        self._target_distance = distance
        self._speed           = speed

    def _run(self) -> str:
        start_x = self._pose.x if self._pose else 0.0
        start_y = self._pose.y if self._pose else 0.0

        msg          = Twist()
        msg.linear.x = self._speed

        self._node.get_logger().info(f"Moving {self._target_distance:.2f} meters...")

        while rclpy.ok():
            current_distance = math.hypot(
                self._pose.x - start_x if self._pose else 0.0,
                self._pose.y - start_y if self._pose else 0.0,
            )
            if current_distance >= self._target_distance:
                break
            self._vel_publisher.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.05)

        msg.linear.x = 0.0
        self._vel_publisher.publish(msg)
        return SUCCEED


class RotateState(TurtleState):
    _TWO_PI = 2 * math.pi

    def __init__(self, target_angle_rad: float, angular_speed: float = 1.0) -> None:
        super().__init__()
        self._target_angle = target_angle_rad
        self._speed        = angular_speed if target_angle_rad > 0 else -angular_speed

    def _run(self) -> str:
        msg          = Twist()
        msg.angular.z = self._speed

        rotated_angle = 0.0
        last_theta    = self._pose.theta if self._pose else 0.0

        self._node.get_logger().info(
            f"Rotating {math.degrees(self._target_angle):.1f} degrees..."
        )

        while rclpy.ok():
            delta = self._pose.theta - last_theta if self._pose else 0.0

            if delta >  math.pi: delta -= self._TWO_PI
            if delta < -math.pi: delta += self._TWO_PI

            rotated_angle += delta
            last_theta     = self._pose.theta if self._pose else last_theta

            if abs(rotated_angle) >= abs(self._target_angle):
                break

            self._vel_publisher.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.05)

        msg.angular.z = 0.0
        self._vel_publisher.publish(msg)
        return SUCCEED


# =========================================================
# MISSION SEQUENCE
# =========================================================

#   A # # # # D
#     #
#   B # # # # E
#     #
#   C #

MISSION: list[MissionStep] = [
    MissionStep("c2a_rotate", StepAction.ROTATE,  math.pi / 2),
    MissionStep("c2a_move",   StepAction.MOVE,    4.0        ),
    MissionStep("a2d_rotate", StepAction.ROTATE, -math.pi / 2),
    MissionStep("a2d_move",   StepAction.MOVE,    2.0        ),
    MissionStep("d2a_rotate", StepAction.ROTATE,  math.pi    ),
    MissionStep("d2a_move",   StepAction.MOVE,    2.0        ),
    MissionStep("a2b_rotate", StepAction.ROTATE,  math.pi / 2),
    MissionStep("a2b_move",   StepAction.MOVE,    2.0        ),
    MissionStep("b2e_rotate", StepAction.ROTATE,  math.pi / 2),
    MissionStep("b2e_move",   StepAction.MOVE,    2.0        ),
]


def _build_state(step: MissionStep) -> State:
    if step.action is StepAction.MOVE:
        return MoveState(distance=step.value, speed=step.speed)
    return RotateState(target_angle_rad=step.value, angular_speed=step.speed)


def _build_state_machine(mission: list[MissionStep]) -> StateMachine:
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    for i, step in enumerate(mission):
        next_outcome = mission[i + 1].name if i < len(mission) - 1 else SUCCEED
        sm.add_state(
            step.name,
            _build_state(step),
            transitions={
                SUCCEED: next_outcome,
                ABORT  : ABORT,
                CANCEL : CANCEL,
            },
        )

    return sm


"""Initializes ROS2, builds and runs the mission state machine."""
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    blackboard = Blackboard()
    sm         = _build_state_machine(MISSION)

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(f"State machine finished with outcome: {outcome}")
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()