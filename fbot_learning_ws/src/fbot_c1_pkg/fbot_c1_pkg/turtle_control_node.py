#!/usr/bin/env python3

# turtlesim
# ros2 run turtlesim turtlesim_node

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PI = 3.1415926535897
speed = 3.0

class ChallengeNode(Node):
    def __init__(self):
        super().__init__('challenge_node')

        self.get_logger().info('ChallengeNode has been started')

        self.vel_pubslisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)        
        self.timer = self.create_timer(0.1, self.execute)

        self.compleated = False
        self.sides = 0


    def move(self, distance=None, angle=None):
        msg = Twist()

        msg.linear.x  = float(speed) if distance else 0.0
        msg.angular.z = float(speed) if angle    else 0.0

        t_linear  = (distance / speed) if distance else 0.0
        t_angular =    (angle / speed) if angle    else 0.0

        target_time = max(t_linear, t_angular)

        t0 = self.get_clock().now().nanoseconds / 1e9

        while (self.get_clock().now().nanoseconds / 1e9 - t0) < target_time:
            self.vel_pubslisher.publish(msg)

        msg.linear.x  = 0.0
        msg.angular.z = 0.0

        self.vel_pubslisher.publish(msg)

    def execute(self):
        '''
        This function is called periodically by the timer.
        It is responsible for controlling the robot's movement.
        It should publish a Twist message to the /cmd_vel topic to control the robot.
        ''' 
        
        if not self.compleated:    
            self.move(distance=2.0)
            self.move(angle=PI/2)

            self.sides += 1

            if self.sides == 4:
                self.compleated = True
                self.get_logger().info('Challenge compleated!')

        else:  
            input('Press Enter to exit...')
            self.compleated = False
            self.sides = 0

def main(args=None):
    rclpy.init(args=args)
    node = ChallengeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()