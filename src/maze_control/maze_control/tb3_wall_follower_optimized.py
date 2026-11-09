#!/usr/bin/env python3
import math
from statistics import median

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class OptimizedLeftWallFollower(Node):
    """基于雷达扇区统计、P 控制和速度自适应的左墙跟随节点。"""

    def __init__(self):
        super().__init__('tb3_wall_follower_optimized')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )

        # 主要控制参数，可根据迷宫通道宽度和实际运行效果调整。
        self.declare_parameter('target_left', 0.45)
        self.declare_parameter('front_danger', 0.45)
        self.declare_parameter('wall_lost_distance', 1.20)
        self.declare_parameter('kp_wall', 1.00)
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('min_linear', 0.10)
        self.declare_parameter('max_angular', 0.70)

        # 使用明确的 double_value 类型，避免编辑器将参数值判断为 None。
        self.target_left = self.get_double_parameter('target_left')
        self.front_danger = self.get_double_parameter('front_danger')
        self.wall_lost_distance = self.get_double_parameter(
            'wall_lost_distance'
        )
        self.kp_wall = self.get_double_parameter('kp_wall')
        self.max_linear = self.get_double_parameter('max_linear')
        self.min_linear = self.get_double_parameter('min_linear')
        self.max_angular = self.get_double_parameter('max_angular')

        self.front = math.inf
        self.left = math.inf
        self.scan_received = False
        self.last_state = None
        self.timer = self.create_timer(0.10, self.control_loop)

        self.get_logger().info('Optimized left-wall follower started')

    def get_double_parameter(self, name: str) -> float:
        """读取已经声明的 ROS 2 浮点参数。"""
        parameter = self.get_parameter(name)
        return parameter.get_parameter_value().double_value

    @staticmethod
    def sector_values(msg, min_deg, max_deg):
        """提取指定角度扇区内满足雷达量程要求的有效距离。"""
        values = []
        min_angle = math.radians(min_deg)
        max_angle = math.radians(max_deg)

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if not msg.range_min < distance <= msg.range_max:
                continue

            # 将角度统一转换到 [-pi, pi]，兼容不同的雷达角度范围。
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if min_angle <= angle <= max_angle:
                values.append(distance)

        return values

    @staticmethod
    def low_percentile(values, fraction=0.10):
        """使用较低分位值代替单点最小值，降低偶发噪声影响。"""
        ordered = sorted(values)
        index = min(len(ordered) - 1, int(len(ordered) * fraction))
        return ordered[index]

    def scan_callback(self, msg):
        # 前方扇区用于避障，左侧扇区用于估计机器人与墙面的距离。
        front_values = self.sector_values(msg, -15.0, 15.0)
        left_values = self.sector_values(msg, 60.0, 100.0)

        self.front = (
            self.low_percentile(front_values)
            if front_values else msg.range_max
        )
        self.left = median(left_values) if left_values else msg.range_max
        self.scan_received = True

    def publish_state(self, state):
        if state != self.last_state:
            self.get_logger().info(
                f'state={state}, front={self.front:.2f} m, '
                f'left={self.left:.2f} m'
            )
            self.last_state = state

    def control_loop(self):
        twist = Twist()

        if not self.scan_received:
            # 尚未收到雷达数据时保持停车，避免机器人盲目运动。
            self.publish_state('waiting_for_scan')
            self.cmd_pub.publish(twist)
            return

        if self.front < self.front_danger:
            # 前方避障优先级最高：停止前进并原地右转。
            twist.linear.x = 0.0
            twist.angular.z = -self.max_angular
            self.publish_state('turn_right')

        elif self.left >= self.wall_lost_distance:
            # 左墙丢失时减速向左搜索，适应左转入口或较大开口。
            twist.linear.x = self.min_linear
            twist.angular.z = 0.55 * self.max_angular
            self.publish_state('search_left_wall')

        else:
            # 墙距误差为正表示离墙过远，P 控制输出正角速度向左靠近。
            wall_error = self.left - self.target_left
            angular = self.kp_wall * wall_error
            twist.angular.z = max(
                -self.max_angular,
                min(self.max_angular, angular),
            )

            # 转向幅度越大，线速度越低，使拐弯和墙距调整更加平滑。
            turn_ratio = min(
                abs(twist.angular.z) / self.max_angular,
                1.0,
            )
            twist.linear.x = self.max_linear - turn_ratio * (
                self.max_linear - self.min_linear
            )
            self.publish_state('follow_wall')

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = OptimizedLeftWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
