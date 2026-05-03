#!/usr/bin/env python3

import rclpy
import yasmin
import time
import math

from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_ros.yasmin_node import YasminNode
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveState(State):
    def __init__(self, distance: float, speed: float = 1.0):
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL])
        self.node = YasminNode.get_instance()
        self.vel_publisher = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose = None
        self.pose_sub = self.node.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.target_distance = distance
        self.speed = speed

    def pose_callback(self, msg):
        self.pose = msg

    def execute(self, blackboard: Blackboard) -> str:

        while self.pose is None and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if not rclpy.ok():
            return ABORT

        start_x = self.pose.x if self.pose else 0.0
        start_y = self.pose.y if self.pose else 0.0
        
        msg = Twist()
        msg.linear.x = self.speed

        self.node.get_logger().info(f"Moving {self.target_distance} meters...")

        while rclpy.ok():
            current_distance = math.hypot(self.pose.x - start_x, self.pose.y - start_y) if self.pose else 0.0
            
            if current_distance >= self.target_distance:
                break
                
            self.vel_publisher.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        msg.linear.x = 0.0
        self.vel_publisher.publish(msg)
        return SUCCEED

class RotateState(State):
    def __init__(self, target_angle_rad: float, angular_speed: float = 1.0):
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL])
        self.node = YasminNode.get_instance()
        self.vel_publisher = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose = None
        self.pose_sub = self.node.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.target_angle = target_angle_rad
        self.speed = angular_speed if target_angle_rad > 0 else -angular_speed

    def pose_callback(self, msg):
        self.pose = msg

    def execute(self, blackboard: Blackboard) -> str:
        while self.pose is None and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if not rclpy.ok():
            return ABORT

        msg = Twist()
        msg.angular.z = self.speed
        
        rotated_angle = 0.0
        last_theta = self.pose.theta if self.pose else 0.0

        self.node.get_logger().info(f"Rotating {math.degrees(self.target_angle):.1f} degrees...")

        while rclpy.ok():
            delta = self.pose.theta - last_theta if self.pose else 0.0
            if delta > math.pi:    
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
                
            rotated_angle += delta
            last_theta = self.pose.theta if self.pose else 0.0

            if abs(rotated_angle) >= abs(self.target_angle):
                break

            self.vel_publisher.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        msg.angular.z = 0.0
        self.vel_publisher.publish(msg)
        return SUCCEED

def main(args=None):
    rclpy.init(args=args)

    blackboard = Blackboard()

    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])


    #   A # # # # D
    #     # 
    #   B # # # # E
    #     #
    #   C #

    sequence = [
        ("c2a_rotate", RotateState(math.pi / 2)),
        ("c2a_move",   MoveState(4.0)),
        ("a2d_rotate", RotateState(-math.pi / 2)),
        ("a2d_move",   MoveState(2.0)),
        ("d2a_rotate", RotateState(math.pi)),
        ("d2a_move",   MoveState(2.0)),
        ("a2b_rotate", RotateState(math.pi / 2)),
        ("a2b_move",   MoveState(2.0)),
        ("b2e_rotate", RotateState(math.pi / 2)),
        ("b2e_move",   MoveState(2.0))
    ]

    for i, (state_name, state_instance) in enumerate(sequence):
        if i == len(sequence) - 1:
            next_state = SUCCEED
        else:
            next_state = sequence[i + 1][0]

        sm.add_state(
            state_name,
            state_instance,
            transitions={
                SUCCEED: next_state,
                ABORT: ABORT,
                CANCEL: CANCEL,
            }
        )

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