#!/usr/bin/env python3

"""
Square trajectory challenge using turtlesim.

DEV     : Gabriel
PROJECT : ROS2 Turtlesim Challenge

USAGE:
    ros2 run turtlesim turtlesim_node
    ros2 run fbot_c1_pkg turtle_control_c1_node 
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ChallengeNode(Node):
    """
    ROS2 node that commands a turtle to trace a square trajectory.

    The node draws a square with 4 sides by alternating forward movements
    and 90-degree left turns. After completing the square, it waits for
    user input before resetting and allowing a new run.
    """

    _PARAM_SPEED      = 'speed'
    _TIMER_PERIOD     = 0.1
    _SQUARE_SIDES     = 4
    _SIDE_DISTANCE    = 2.0
    _TURN_ANGLE       = math.pi / 2

    def __init__(self) -> None:
        super().__init__('challenge_1_node')

        self.declare_parameter(self._PARAM_SPEED, 3.0)

        self._vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._timer         = self.create_timer(self._TIMER_PERIOD, self._execute)

        self._completed = False
        self._sides     = 0

        self.get_logger().info('ChallengeNode has been started.')

    @property
    def _speed(self) -> float:
        """Reads the current speed parameter from the ROS2 parameter server."""
        return self.get_parameter(self._PARAM_SPEED).get_parameter_value().double_value

    def _move(self, distance: float | None = None, angle: float | None = None) -> None:
        """
        Publishes velocity commands to move the turtle for a calculated duration.

        Computes the required time from distance/angle and speed, then
        busy-waits while publishing the command. Sends a stop command after.

        Args:
            distance: Linear distance to travel (meters).
            angle:    Angular displacement to rotate (radians).
        """
        speed = self._speed
        msg   = Twist()

        msg.linear.x  = float(speed) if distance else 0.0
        msg.angular.z = float(speed) if angle    else 0.0

        t_linear  = (distance / speed) if distance else 0.0
        t_angular = (angle    / speed) if angle    else 0.0

        target_time = max(t_linear, t_angular)
        t0          = self.get_clock().now().nanoseconds / 1e9

        while (self.get_clock().now().nanoseconds / 1e9 - t0) < target_time:
            self._vel_publisher.publish(msg)

        msg.linear.x  = 0.0
        msg.angular.z = 0.0
        self._vel_publisher.publish(msg)

    def _execute(self) -> None:
        """
        Timer callback that drives the square trajectory state machine.

        Called periodically at `_TIMER_PERIOD` seconds. On each call,
        commands one side and one turn until the square is complete.
        """
        if not self._completed:
            self._move(distance=self._SIDE_DISTANCE)
            self._move(angle=self._TURN_ANGLE)

            self._sides += 1

            if self._sides == self._SQUARE_SIDES:
                self._completed = True
                self.get_logger().info('Challenge completed!')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ChallengeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()