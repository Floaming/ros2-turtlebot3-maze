#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi, sin, cos

class WaypointController(Node):
    def __init__(self):
        super().__init__('tb3_waypoints')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.Kp_dist = 0.3
        self.Kp_ang = 0.8
        self.max_v = 0.2
        self.max_w = 1.0
        self.threshold = 0.05   # 航点到达切换阈值

        # 矩形航点列表，可自行修改
        self.waypoints = [(1.0,0.0), (1.0,1.0), (0.0,1.0), (0.0,0.0)]
        self.wp_index = 0

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
            self.get_logger().info("全部航点完成，停车")
            self.cmd_pub.publish(Twist())
            return
        gx, gy = self.waypoints[self.wp_index]
        dx = gx - self.x
        dy = gy - self.y
        dist_err = sqrt(dx**2 + dy**2)
        theta_des = atan2(dy, dx)
        theta_err = theta_des - self.theta
        theta_err = atan2(sin(theta_err), cos(theta_err))

        twist = Twist()
        if dist_err > self.threshold:
            twist.linear.x = self.Kp_dist * dist_err
            twist.angular.z = self.Kp_ang * theta_err
            twist.linear.x = max(-self.max_v, min(self.max_v, twist.linear.x))
            twist.angular.z = max(-self.max_w, min(self.max_w, twist.angular.z))
        else:
            self.get_logger().info(f"到达航点{self.wp_index}:({gx},{gy})，切换下一个航点")
            self.wp_index += 1
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()