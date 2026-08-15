#!/usr/bin/env python3
"""
分析 /scan topic，找出固定近距離回波（可能是車體上的手臂控制箱）。
收集 10 筆 scan 取平均，列出每個角度的平均距離，
標記出可能需要過濾的區段。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__('scan_analyzer')
        self.samples = []
        self.max_samples = 10
        self.scan_info = None
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.get_logger().info('等待 /scan 資料...')

    def cb(self, msg: LaserScan):
        if self.scan_info is None:
            self.scan_info = {
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'num_ranges': len(msg.ranges),
            }
            self.get_logger().info(
                f"Scan info: angle_min={math.degrees(msg.angle_min):.1f}°, "
                f"angle_max={math.degrees(msg.angle_max):.1f}°, "
                f"increment={math.degrees(msg.angle_increment):.3f}°, "
                f"num_ranges={len(msg.ranges)}"
            )

        self.samples.append(list(msg.ranges))
        self.get_logger().info(f'收集到第 {len(self.samples)}/{self.max_samples} 筆 scan')

        if len(self.samples) >= self.max_samples:
            self.analyze()
            raise SystemExit(0)

    def analyze(self):
        info = self.scan_info
        data = np.array(self.samples)  # (N, num_ranges)

        # Replace inf/nan with range_max for averaging
        data_clean = np.where(np.isfinite(data), data, np.nan)
        mean_ranges = np.nanmean(data_clean, axis=0)

        # 定義「近距離」門檻 — 小於此距離視為可能的車體障礙物
        CLOSE_THRESHOLD = 0.5  # 0.5m 以內
        VERY_CLOSE = 0.3       # 0.3m 以內 (幾乎確定是車體)

        num = len(mean_ranges)
        angles_deg = [
            math.degrees(info['angle_min'] + i * info['angle_increment'])
            for i in range(num)
        ]

        print("\n" + "=" * 70)
        print("  SCAN 分析結果 — 近距離回波區段（可能是手臂控制箱）")
        print("=" * 70)
        print(f"  掃描範圍: {math.degrees(info['angle_min']):.1f}° ~ "
              f"{math.degrees(info['angle_max']):.1f}°")
        print(f"  總光束數: {num}")
        print(f"  近距離門檻: < {CLOSE_THRESHOLD}m")
        print("=" * 70)

        # 找出連續的近距離區段
        in_segment = False
        segments = []  # list of (start_deg, end_deg, min_dist)
        seg_start = None
        seg_min_dist = float('inf')

        for i in range(num):
            r = mean_ranges[i]
            deg = angles_deg[i]

            if np.isfinite(r) and r < CLOSE_THRESHOLD:
                if not in_segment:
                    seg_start = deg
                    seg_min_dist = r
                    in_segment = True
                else:
                    seg_min_dist = min(seg_min_dist, r)
            else:
                if in_segment:
                    segments.append((seg_start, angles_deg[i - 1], seg_min_dist))
                    in_segment = False
                    seg_min_dist = float('inf')

        if in_segment:
            segments.append((seg_start, angles_deg[num - 1], seg_min_dist))

        if not segments:
            print("\n  ✅ 沒有發現 < 0.5m 的固定近距離回波。")
            print("  嘗試將門檻調高或檢查 LiDAR 是否有資料。")
        else:
            print(f"\n  找到 {len(segments)} 個近距離區段:\n")
            print(f"  {'區段':>4}  {'起始角度':>10}  {'結束角度':>10}  "
                  f"{'角度寬度':>10}  {'最近距離':>10}  {'起始(rad)':>10}  {'結束(rad)':>10}")
            print("  " + "-" * 78)

            for idx, (s, e, d) in enumerate(segments, 1):
                width = e - s
                s_rad = math.radians(s)
                e_rad = math.radians(e)
                marker = " ⚠️ 建議過濾" if d < VERY_CLOSE else ""
                print(f"  {idx:>4}  {s:>9.1f}°  {e:>9.1f}°  "
                      f"{width:>9.1f}°  {d:>9.3f}m  "
                      f"{s_rad:>10.4f}  {e_rad:>10.4f}{marker}")

        # 也列出整圈的距離分佈圖（以 10° 為單位）
        print("\n" + "=" * 70)
        print("  角度 vs 平均距離 (每 10° 區間的最小距離)")
        print("=" * 70)
        print(f"  {'角度範圍':>16}  {'最小距離':>10}  {'圖示'}")
        print("  " + "-" * 60)

        step = 10
        angle_start = int(math.degrees(info['angle_min']))
        angle_end = int(math.degrees(info['angle_max']))

        for a in range(angle_start, angle_end, step):
            # Find indices in this angle range
            min_r = float('inf')
            for i in range(num):
                deg = angles_deg[i]
                if a <= deg < a + step:
                    r = mean_ranges[i]
                    if np.isfinite(r) and r < min_r:
                        min_r = r

            if min_r == float('inf'):
                bar = "  (no data)"
            else:
                bar_len = min(int(min_r * 10), 40)  # 每 0.1m 一格, max 40
                bar = "█" * bar_len if bar_len > 0 else "▏"
                if min_r < CLOSE_THRESHOLD:
                    bar += f"  ← 近! ({min_r:.2f}m)"

            print(f"  {a:>4}° ~ {a+step:>4}°  {min_r:>9.2f}m  {bar}")

        print("\n" + "=" * 70)
        print("  建議: 將上述標記 '⚠️ 建議過濾' 的角度範圍加入 laser_filter.yaml")
        print("=" * 70 + "\n")


def main():
    rclpy.init()
    node = ScanAnalyzer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
