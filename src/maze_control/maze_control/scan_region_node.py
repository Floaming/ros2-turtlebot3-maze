#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanRegionFilter(Node):
    def __init__(self):
        super().__init__('scan_region_filter_node')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        # 角度配置（单位：度）
        self.front_min_deg = -15
        self.front_max_deg = 15

        self.left_min_deg = 60
        self.left_max_deg = 90

        self.right_min_deg = -90
        self.right_max_deg = -60

        self.get_logger().info("区域雷达节点启动：前方[-15,15]°, 左[60,90]°, 右[-90,-60]°")

    def deg2rad(self, deg):
        return deg * np.pi / 180.0

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = np.array(msg.ranges)
        max_range = msg.range_max

        # 有效掩码：非inf、非nan、在量程内
        valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges <= max_range)

        # -------- 前方区域 [-15°,15°]，取最小值 --------
        a0 = self.deg2rad(self.front_min_deg)
        a1 = self.deg2rad(self.front_max_deg)
        idx_front = np.where(
            (angle_min + np.arange(len(ranges)) * angle_inc >= a0) &
            (angle_min + np.arange(len(ranges)) * angle_inc <= a1) & valid_mask
        )[0]
        front_dist = np.min(ranges[idx_front]) if len(idx_front) > 0 else max_range

        # -------- 左侧区域 [60°,90°]，取中值 --------
        a0 = self.deg2rad(self.left_min_deg)
        a1 = self.deg2rad(self.left_max_deg)
        idx_left = np.where(
            (angle_min + np.arange(len(ranges)) * angle_inc >= a0) &
            (angle_min + np.arange(len(ranges)) * angle_inc <= a1) & valid_mask
        )[0]
        left_dist = np.median(ranges[idx_left]) if len(idx_left) > 0 else max_range

        # -------- 右侧区域 [-90°,-60°]，取中值 --------
        a0 = self.deg2rad(self.right_min_deg)
        a1 = self.deg2rad(self.right_max_deg)
        idx_right = np.where(
            (angle_min + np.arange(len(ranges)) * angle_inc >= a0) &
            (angle_min + np.arange(len(ranges)) * angle_inc <= a1) & valid_mask
        )[0]
        right_dist = np.median(ranges[idx_right]) if len(idx_right) > 0 else max_range

        self.get_logger().info(
            f"【距离】前方: {front_dist:.2f} m | 左侧: {left_dist:.2f} m | 右侧: {right_dist:.2f} m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ScanRegionFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
