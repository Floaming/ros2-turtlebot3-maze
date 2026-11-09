#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi, sin, cos

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('tb3_pure_pursuit')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.waypoints = [(1.0,0.0), (1.0,1.0), (0.0,1.0), (0.0,0.0)]
        self.wp_index = 0
        self.Ld = 0.35          # 前瞻距离 m
        self.max_v = 0.2        # 恒速前进
        self.max_w = 1.0
        self.arrive_threshold = 0.1

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.wp_index >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            return
        gx, gy = self.waypoints[self.wp_index]
        dx = gx - self.x
        dy = gy - self.y
        dist = sqrt(dx**2 + dy**2)
        if dist < self.arrive_threshold:
            self.get_logger().info(f"到达航点{self.wp_index}，切换")
            self.wp_index += 1
            return
        # 前瞻角：当前航点相对机器人航向的夹角
        alpha = atan2(dy, dx) - self.theta
        alpha = atan2(sin(alpha), cos(alpha))
        twist = Twist()
        twist.linear.x = self.max_v
        # 纯追踪曲率公式 omega = 2*v*sin(alpha)/Ld
        omega = 2.0 * self.max_v * sin(alpha) / self.Ld
        twist.angular.z = max(-self.max_w, min(self.max_w, omega))
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()