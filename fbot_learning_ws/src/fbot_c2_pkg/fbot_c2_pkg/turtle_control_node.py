#!/usr/bin/env python3
"""
Bug2 algorithm for robot motion planning.

DEV     : Gabriel
PROJECT : ROS2 Bug2 Challenge

REFERENCE:
    https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/

USAGE:
    ros2 run <your_package> bug2_node
"""

import math
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass
class Pose:
    x    : float = 0.0
    y    : float = 0.0
    theta: float = 0.0


@dataclass(frozen=True)
class Goal:
    x: float
    y: float


@dataclass
class LaserRegions:
    front: float = 10.0
    left : float = 10.0
    right: float = 10.0


class ChallengeNode(Node):
    _GOAL_TOLERANCE        : float = 0.10
    _YAW_TOLERANCE         : float = 0.15   # ~8.5 degrees
    _OBSTACLE_FRONT_STOP   : float = 0.60
    _WALL_FRONT_STOP       : float = 0.50
    _WALL_RIGHT_TOO_CLOSE  : float = 0.30
    _WALL_RIGHT_TOO_FAR    : float = 0.60
    _M_LINE_TOLERANCE      : float = 0.15
    _HIT_DISTANCE_MARGIN   : float = 0.20

    # --- Velocity commands ---
    _ANGULAR_SPEED         : float = 0.50
    _LINEAR_SPEED          : float = 0.50
    _WALL_LINEAR_SPEED     : float = 0.40
    _WALL_ADJUST_LINEAR    : float = 0.30
    _WALL_ADJUST_ANGULAR   : float = 0.40
    _WALL_TURN_ANGULAR     : float = 0.60

    # --- Laser arc indices ---
    _FRONT_HALF_ARC        : int = 60
    _LEFT_ARC_INNER        : int = 80
    _LEFT_ARC_OUTER        : int = 100
    _RIGHT_ARC_INNER       : int = 80
    _RIGHT_ARC_OUTER       : int = 100

    # --- Timer ---
    _TIMER_PERIOD          : float = 0.10

    def __init__(self) -> None:
        super().__init__('challenge_node')

        self.odom_sub      = self.create_subscription(Odometry,  '/odom',       self.odom_callback,  10)
        self.laser_sub     = self.create_subscription(LaserScan, '/base_scan',   self.laser_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(self._TIMER_PERIOD, self.execute)

        theta_init_rad       = math.radians(45.0)
        self.world_pose_init = Pose(x=-7.0, y=-7.0, theta=theta_init_rad)
        self.world_pose      = Pose(x=-7.0, y=-7.0, theta=theta_init_rad)

        self.goal    = Goal(x=4.5, y=4.0)
        self.state   = 'GO_TO_GOAL'
        self.regions = LaserRegions()

        self.hit_distance_to_goal: float = 0.0

        self.get_logger().info('ChallengeNode (Bug2) has been started.')


    def _get_yaw_from_quaternion(self, q) -> float:
        """Extracts yaw angle from a ROS2 quaternion message."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _transform_to_world_frame(
        self, local_x: float, local_y: float, local_theta: float
    ) -> tuple[float, float, float]:
        ti = self.world_pose_init.theta
        wx = self.world_pose_init.x + local_x * math.cos(ti) - local_y * math.sin(ti)
        wy = self.world_pose_init.y + local_x * math.sin(ti) + local_y * math.cos(ti)
        wt = math.atan2(math.sin(ti + local_theta), math.cos(ti + local_theta))
        return wx, wy, wt

    def _distance_to_point(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    def _distance_to_line(
        self,
        x0: float, y0: float,
        x_start: float, y_start: float,
        x_goal: float, y_goal: float,
    ) -> float:
        num = abs((y_goal - y_start) * x0 - (x_goal - x_start) * y0 + x_goal * y_start - y_goal * x_start)
        den = math.hypot(y_goal - y_start, x_goal - x_start)
        return num / den if den != 0.0 else 0.0


    def odom_callback(self, msg: Odometry) -> None:
        """Updates world-frame pose from odometry."""
        local_x     = msg.pose.pose.position.x
        local_y     = msg.pose.pose.position.y
        local_theta = self._get_yaw_from_quaternion(msg.pose.pose.orientation)

        wx, wy, wt = self._transform_to_world_frame(local_x, local_y, local_theta)
        self.world_pose.x     = wx
        self.world_pose.y     = wy
        self.world_pose.theta = wt

    def laser_callback(self, msg: LaserScan) -> None:
        """Partitions laser scan into front, left, and right region minimums."""
        ranges = [
            r if not math.isinf(r) and not math.isnan(r) and r > 0.05 else 10.0
            for r in msg.ranges
        ]

        size = len(ranges)
        if size == 0:
            return

        mid = size // 2

        front_arc = ranges[mid - self._FRONT_HALF_ARC  : mid + self._FRONT_HALF_ARC]
        left_arc  = ranges[mid + self._LEFT_ARC_INNER  : size - self._LEFT_ARC_OUTER]
        right_arc = ranges[self._RIGHT_ARC_OUTER        : mid - self._RIGHT_ARC_INNER]

        self.regions.front = min(front_arc) if front_arc else 10.0
        self.regions.left  = min(left_arc)  if left_arc  else 10.0
        self.regions.right = min(right_arc) if right_arc else 10.0


    def _go_to_goal(self, msg: Twist, dist_to_goal: float) -> None:
        """GO_TO_GOAL state: steers toward the target, switches to wall-following on obstacle."""
        angle_to_goal = math.atan2(
            self.goal.y - self.world_pose.y,
            self.goal.x - self.world_pose.x,
        )
        yaw_error = math.atan2(
            math.sin(angle_to_goal - self.world_pose.theta),
            math.cos(angle_to_goal - self.world_pose.theta),
        )

        if abs(yaw_error) > self._YAW_TOLERANCE:
            msg.linear.x  = 0.0
            msg.angular.z = self._ANGULAR_SPEED if yaw_error > 0 else -self._ANGULAR_SPEED
        else:
            if self.regions.front < self._OBSTACLE_FRONT_STOP:
                self.state                = 'WALL_FOLLOWING'
                self.hit_distance_to_goal = dist_to_goal
                self.get_logger().info('Obstacle ahead! Switching to WALL_FOLLOWING.')
            else:
                msg.linear.x  = self._LINEAR_SPEED
                msg.angular.z = 0.0

    def _wall_following(self, msg: Twist, dist_to_goal: float) -> None:
        """WALL_FOLLOWING state: follows wall contour, returns to GO_TO_GOAL on M-line crossing."""
        if self.regions.front < self._WALL_FRONT_STOP:
            msg.linear.x  = 0.0
            msg.angular.z = self._WALL_TURN_ANGULAR
        elif self.regions.right < self._WALL_RIGHT_TOO_CLOSE:
            msg.linear.x  = self._WALL_ADJUST_LINEAR
            msg.angular.z = self._WALL_ADJUST_ANGULAR
        elif self.regions.right > self._WALL_RIGHT_TOO_FAR:
            msg.linear.x  = self._WALL_ADJUST_LINEAR
            msg.angular.z = -self._WALL_ADJUST_ANGULAR
        else:
            msg.linear.x  = self._WALL_LINEAR_SPEED
            msg.angular.z = 0.0

        dist_to_m_line = self._distance_to_line(
            self.world_pose.x,     self.world_pose.y,
            self.world_pose_init.x, self.world_pose_init.y,
            self.goal.x,            self.goal.y,
        )

        closer_to_goal = dist_to_goal < (self.hit_distance_to_goal - self._HIT_DISTANCE_MARGIN)
        if dist_to_m_line < self._M_LINE_TOLERANCE and closer_to_goal:
            self.state = 'GO_TO_GOAL'
            self.get_logger().info('Crossed M-Line closer to goal! Switching to GO_TO_GOAL.')

    def execute(self) -> None:
        msg          = Twist()
        dist_to_goal = self._distance_to_point(
            self.world_pose.x, self.world_pose.y,
            self.goal.x,       self.goal.y,
        )

        if dist_to_goal < self._GOAL_TOLERANCE:
            self.get_logger().info('Goal reached!')
            self.vel_publisher.publish(msg)
            return

        if   self.state == 'GO_TO_GOAL'    : self._go_to_goal(msg, dist_to_goal)
        elif self.state == 'WALL_FOLLOWING' : self._wall_following(msg, dist_to_goal)

        self.vel_publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Initializes the ROS2 runtime and spins the ChallengeNode."""
    rclpy.init(args=args)
    node = ChallengeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()