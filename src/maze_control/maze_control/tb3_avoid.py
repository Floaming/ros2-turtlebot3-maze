#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data


class SimpleAvoidNode(Node):
    def __init__(self):
        super().__init__("lidar_simple_avoid")

        # 发布速度话题 /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # 订阅雷达 /scan，sensor_data QoS 适配雷达
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 参数，可以自己调
        self.safe_distance = 0.8    # 安全阈值(m)，小于这个值认为前方有障碍
        self.forward_speed = 0.15   # 前进速度 m/s
        self.turn_speed = 0.3       # 转向角速度 rad/s

        self.get_logger().info("简单避障节点启动：无障碍物前进，遇障碍右转！")

    def scan_callback(self, msg: LaserScan):
        """雷达回调函数"""
        total_points = len(msg.ranges)
        # 360度雷达，正前方是中间位置；取正前方 ±15° 扇形区域判断障碍物
        view_range = int(total_points / 12)
        mid_idx = total_points // 2
        start_idx = mid_idx - view_range
        end_idx = mid_idx + view_range

        # 提取这个扇区内合法距离，过滤inf、超出量程、非法值
        front_ranges = []
        for i in range(start_idx, end_idx):
            d = msg.ranges[i]
            if msg.range_min < d < msg.range_max:
                front_ranges.append(d)

        if len(front_ranges) == 0:
            # 全部无穷远，前方空旷
            min_front_dist = float("inf")
        else:
            min_front_dist = min(front_ranges)

        twist = Twist()
        if min_front_dist > self.safe_distance:
            # 前方空旷：前进
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            self.get_logger().info(f"前方空旷，最近距离:{min_front_dist:.2f} m → 前进")
        else:
            # 前方有障碍物：原地右转
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.get_logger().info(f"前方检测到障碍:{min_front_dist:.2f} m → 右转")

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """退出时调用，给小车发停止指令"""
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        self.get_logger().info("节点退出，小车停止")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
