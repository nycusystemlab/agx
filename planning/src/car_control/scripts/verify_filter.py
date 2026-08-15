#!/usr/bin/env python3
"""比對 /scan vs /scan_filtered，驗證過濾效果。"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class FilterVerifier(Node):
    def __init__(self):
        super().__init__('filter_verifier')
        self.raw = None
        self.filtered = None
        self.sub_raw = self.create_subscription(LaserScan, '/scan', self.cb_raw, 10)
        self.sub_filt = self.create_subscription(LaserScan, '/scan_filtered', self.cb_filt, 10)
        self.get_logger().info('等待 /scan 和 /scan_filtered ...')

    def cb_raw(self, msg):
        self.raw = msg
        self._try_compare()

    def cb_filt(self, msg):
        self.filtered = msg
        self._try_compare()

    def _try_compare(self):
        if self.raw is None or self.filtered is None:
            return

        raw = np.array(self.raw.ranges)
        filt = np.array(self.filtered.ranges)
        n = len(raw)

        angles_deg = [
            math.degrees(self.raw.angle_min + i * self.raw.angle_increment)
            for i in range(n)
        ]

        # Count valid (finite & > 0) points
        raw_valid = np.isfinite(raw) & (raw > 0)
        filt_valid = np.isfinite(filt) & (filt > 0)

        removed = raw_valid & ~filt_valid  # points that were filtered out

        print("\n" + "=" * 60)
        print("  /scan vs /scan_filtered 比對結果")
        print("=" * 60)
        print(f"  總光束數:          {n}")
        print(f"  /scan 有效點數:    {np.sum(raw_valid)}")
        print(f"  /scan_filtered:    {np.sum(filt_valid)}")
        print(f"  被過濾掉的點數:    {np.sum(removed)}")
        print("=" * 60)

        if np.sum(removed) > 0:
            # Find angular range of removed points
            removed_angles = [angles_deg[i] for i in range(n) if removed[i]]
            print(f"  過濾角度範圍: {min(removed_angles):.1f}° ~ {max(removed_angles):.1f}°")
            print("=" * 60)

            # Show 10-degree buckets
            print(f"\n  {'角度範圍':>16}  {'原始':>6}  {'過濾後':>6}  {'移除':>6}  {'狀態'}")
            print("  " + "-" * 55)
            step = 10
            for a in range(int(math.degrees(self.raw.angle_min)),
                           int(math.degrees(self.raw.angle_max)), step):
                r_cnt = 0
                f_cnt = 0
                rm_cnt = 0
                for i in range(n):
                    d = angles_deg[i]
                    if a <= d < a + step:
                        if raw_valid[i]:
                            r_cnt += 1
                        if filt_valid[i]:
                            f_cnt += 1
                        if removed[i]:
                            rm_cnt += 1
                status = "✅ 已過濾" if rm_cnt > 0 else "  正常"
                print(f"  {a:>4}° ~ {a+step:>4}°  {r_cnt:>6}  {f_cnt:>6}  {rm_cnt:>6}  {status}")

            print("\n  ✅ laser_filter 運作正常！")
        else:
            print("  ⚠️ 沒有偵測到被過濾的點，請檢查 filter 設定")

        print("=" * 60 + "\n")
        raise SystemExit(0)


def main():
    rclpy.init()
    node = FilterVerifier()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
