#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class ChallengeNode(Node):
    def __init__(self):
        super().__init__('challenge_node')
        self.odom_sub   = self.create_subscription(Odometry, '/odom', self.odomCallback, 10)
        self.laser_sub  = self.create_subscription(LaserScan, '/base_scan', self.laserCallback, 10)
        
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.execute)
        
        theta_init_rad = math.radians(45.0)
        self.world_pose_init = {"x": -7.0, "y": -7.0, "theta": theta_init_rad}
        self.world_pose      = {"x": -7.0, "y": -7.0, "theta": theta_init_rad}

        # Bug 2
        self.goal  = {"x": 4.5, "y": 4.0}  
        self.state = 'GO_TO_GOAL'         
        self.hit_distance_to_goal = 0.0   
        
        self.regions = {
            'front': 10.0,
            'left':  10.0,
            'right': 10.0
        }

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp) 

    def transform_to_world_frame(self, local_x, local_y, local_theta):
        theta_init = self.world_pose_init["theta"]
        
        world_x = self.world_pose_init["x"] + (local_x * math.cos(theta_init)) - (local_y * math.sin(theta_init))
        world_y = self.world_pose_init["y"] + (local_x * math.sin(theta_init)) + (local_y * math.cos(theta_init))
        
        world_theta = theta_init + local_theta
        world_theta = math.atan2(math.sin(world_theta), math.cos(world_theta))
        
        return world_x, world_y, world_theta

    def distance_to_point(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def distance_to_line(self, x0, y0, x_start, y_start, x_goal, y_goal):
        num = abs((y_goal - y_start)*x0 - (x_goal - x_start)*y0 + x_goal*y_start - y_goal*x_start)
        den = math.sqrt((y_goal - y_start)**2 + (x_goal - x_start)**2)
        return num / den if den != 0 else 0.0

    def odomCallback(self, msg):
        local_x = msg.pose.pose.position.x
        local_y = msg.pose.pose.position.y
        local_theta = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

        self.world_pose["x"], self.world_pose["y"], self.world_pose["theta"] = self.transform_to_world_frame(
            local_x, local_y, local_theta
        )

    def laserCallback(self, msg):
        ranges = [r if not math.isinf(r) and not math.isnan(r) and r > 0.05 else 10.0 for r in msg.ranges]
        
        size = len(ranges)
        if size == 0:
            return

        mid = size // 2
     
        front_arc = ranges[mid - 60 : mid + 60]   
        left_arc  = ranges[mid + 80 : size - 100]
        right_arc = ranges[100 : mid - 80]

        self.regions = {
            'front': min(front_arc) if len(front_arc) > 0 else 10.0,
            'left' : min(left_arc)  if len(left_arc)  > 0 else 10.0,
            'right': min(right_arc) if len(right_arc) > 0 else 10.0,
        }

    def execute(self):
        msg = Twist()
        dist_to_goal = self.distance_to_point(
            self.world_pose["x"], self.world_pose["y"], self.goal["x"], self.goal["y"]
        )
        
        if dist_to_goal < 0.1:
            self.get_logger().info('Objetivo Alcançado!')
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_publisher.publish(msg)
            return

        if self.state == 'GO_TO_GOAL':
            angle_to_goal = math.atan2(self.goal["y"] - self.world_pose["y"], self.goal["x"] - self.world_pose["x"])
            yaw_error = angle_to_goal - self.world_pose["theta"]
            
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            
            if abs(yaw_error) > 0.15: # ~8.5 graus de tolerância
                msg.linear.x = 0.0    # Zera a velocidade para não atropelar a parede sem querer
                msg.angular.z = 0.5 if yaw_error > 0 else -0.5
                
            else:
                if self.regions['front'] < 0.6:
                    self.state = 'WALL_FOLLOWING'
                    self.hit_distance_to_goal = dist_to_goal
                    self.get_logger().info('Obstáculo no caminho do alvo! Mudando para WALL_FOLLOWING.')
                else:
                    msg.linear.x = 0.5
                    msg.angular.z = 0.0

        elif self.state == 'WALL_FOLLOWING':
            if self.regions['front'] < 0.5:
                msg.linear.x = 0.0
                msg.angular.z = 0.6
            elif self.regions['right'] < 0.3:
                msg.linear.x = 0.3
                msg.angular.z = 0.4
            elif self.regions['right'] > 0.6:
                msg.linear.x = 0.3
                msg.angular.z = -0.4
            else:
                msg.linear.x = 0.4
                msg.angular.z = 0.0


            dist_to_m_line = self.distance_to_line(
                self.world_pose["x"], self.world_pose["y"], 
                self.world_pose_init["x"], self.world_pose_init["y"], 
                self.goal["x"], self.goal["y"]
            )
            
            if dist_to_m_line < 0.15 and dist_to_goal < (self.hit_distance_to_goal - 0.2):
                self.state = 'GO_TO_GOAL'
                self.get_logger().info('Cruzou a M-Line mais perto do alvo! Voltando para GO_TO_GOAL.')

        self.vel_publisher.publish(msg)


def main(args=None):
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