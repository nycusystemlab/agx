#!/usr/bin/env python3
"""Isaac Sim 的替身資料來源，用來在沒有 Isaac 的情況下端到端驗 nav2。

Isaac Sim 只講 DDS，`rmw_zenoh` 與 DDS 之間沒有橋，所以 planning 切到
`rmw_zenoh_cpp` 之後就收不到 Isaac 的資料。這支節點補上那一段：它發 /clock、
/odom（含 odom→base_footprint TF）與 /scan，並訂閱 /cmd_vel 做差速積分，
所以導航目標真的會被走完，不是靜止替身。/scan 是對 sim_map.pgm 做 ray-cast
得到的，AMCL 才有特徵可比對——餵全空曠的掃描會讓 AMCL 只靠里程計外插而漂走。

無碰撞物理：車會穿牆。它只用於驗通訊與節點圖，不能拿來驗避障。

`planning/tools/` 沒有被 bind-mount 進 planning 容器，所以要先 copy 進去：

    docker cp planning/tools/sim_standin.py planning:/root/
    docker exec -d planning bash -lc 'source /opt/ros/humble/setup.bash \
        && source /root/ros2_ws/install/setup.bash && python3 /root/sim_standin.py'
    STANDIN_MAP=/path/to/map.yaml   # 換地圖

⚠️ **每次重啟這支節點，/clock 都會從 0 重來**，等同 Isaac Sim 重啟的時鐘跳躍：
`ekf_filter_node` 會無限拒收 /odom（`timestamp before that of the previous message`），
`/odometry/filtered` 卡在重啟前的狀態但 /odom 仍有 30 Hz。所以重啟它之後
**一定要連 `car_sensor_sim.launch.py` 與 nav2 一起重啟**。
"""
import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

MAP_YAML = os.environ.get(
    'STANDIN_MAP',
    '/root/ros2_ws/install/car_control/share/car_control/config/sim_map.yaml',
)
LASER_X = 0.41           # base_footprint → laser，取自 amr_core.urdf.xacro
RANGE_MAX = 10.0
RAY_STEP = 0.04

CLOCK_HZ = 100.0
ODOM_HZ = 30.0
SCAN_HZ = 10.0
SCAN_POINTS = 360
CMD_TIMEOUT = 0.5        # 對齊 Isaac Sim DifferentialController 的 watchdog


def load_map(yaml_path):
    """讀 map_server 那組 pgm + yaml，回傳 (occupied 布林陣列, resolution, origin)。"""
    import yaml as _yaml
    with open(yaml_path) as f:
        meta = _yaml.safe_load(f)
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(pgm, 'rb') as f:
        def tok():
            t = b''
            while True:
                c = f.read(1)
                if not c:                       # EOF：b'' 會通過下面的空白字元判斷
                    if t:
                        return t
                    raise ValueError(f'{pgm}: PGM header 未完成就遇到檔案結尾')
                if c in b' \t\r\n':
                    if t:
                        return t
                    continue
                if c == b'#':
                    while True:
                        c = f.read(1)
                        if not c or c in b'\r\n':
                            break
                    continue
                t += c
        assert tok() == b'P5'
        w, h = int(tok()), int(tok())
        tok()
        raw = f.read(w * h)
        if len(raw) != w * h:
            raise ValueError(f'{pgm}: 像素資料只有 {len(raw)} bytes，{w}x{h} 需要 {w * h}')
        px = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
    # trinary：p = (255 - value)/255，p > occupied_thresh 才算佔據
    thresh = 255 * (1.0 - float(meta['occupied_thresh']))
    occ = px < thresh
    return occ[::-1], float(meta['resolution']), meta['origin']


class SimStandin(Node):
    def __init__(self):
        super().__init__('sim_standin')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=False)])

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.tf_bc = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)

        self.occ, self.res, self.origin = load_map(MAP_YAML)
        self.n_steps = int(RANGE_MAX / RAY_STEP)
        self.ray_d = np.arange(1, self.n_steps + 1) * RAY_STEP
        self.ray_a = np.linspace(-math.pi, math.pi, SCAN_POINTS, endpoint=False)

        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.last_cmd = 0.0
        self.t0 = time.monotonic()
        self.sim_t = 0.0

        self.create_timer(1.0 / CLOCK_HZ, self._tick_clock)
        self.create_timer(1.0 / ODOM_HZ, self._tick_odom)
        self.create_timer(1.0 / SCAN_HZ, self._tick_scan)

    def _on_cmd(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd = time.monotonic()

    def _stamp(self):
        sec = int(self.sim_t)
        return sec, int((self.sim_t - sec) * 1e9)

    def _tick_clock(self):
        mono = time.monotonic()
        # 沒有 watchdog 的話，nav2 被 Ctrl-C 掉時最後一筆非零指令會一直積分下去，
        # 車開到地圖外，症狀看起來像 AMCL 發散
        if self.v or self.w:
            if mono - self.last_cmd > CMD_TIMEOUT:
                self.v = self.w = 0.0
        now = mono - self.t0
        dt = now - self.sim_t
        self.sim_t = now
        # 差速積分放在時鐘迴圈：它是頻率最高的一個，積分誤差最小
        self.th += self.w * dt
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        sec, nsec = self._stamp()
        m = Clock()
        m.clock.sec, m.clock.nanosec = sec, nsec
        self.clock_pub.publish(m)

    def _tick_odom(self):
        sec, nsec = self._stamp()
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)

        o = Odometry()
        o.header.stamp.sec, o.header.stamp.nanosec = sec, nsec
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.w
        for i in (0, 7, 14, 21, 28, 35):
            o.pose.covariance[i] = 0.001
            o.twist.covariance[i] = 0.001
        self.odom_pub.publish(o)

        t = TransformStamped()
        t.header.stamp.sec, t.header.stamp.nanosec = sec, nsec
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_bc.sendTransform(t)

    def _raycast(self):
        """沿每條射線等距取樣查佔據格；第一個命中的取樣點就是 range。"""
        ox = self.x + LASER_X * math.cos(self.th)
        oy = self.y + LASER_X * math.sin(self.th)
        ang = self.ray_a + self.th
        xs = ox + np.outer(np.cos(ang), self.ray_d)
        ys = oy + np.outer(np.sin(ang), self.ray_d)

        # 一定要 floor：astype 是向零截斷，origin 左／下各一格會被算成格 0 而誤判為地圖內
        cx = np.floor((xs - self.origin[0]) / self.res).astype(np.int32)
        cy = np.floor((ys - self.origin[1]) / self.res).astype(np.int32)
        h, w = self.occ.shape
        inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        hit = np.zeros_like(inside)
        hit[inside] = self.occ[cy[inside], cx[inside]]
        # 地圖外一律視為命中，否則射線會穿出地圖邊界回報無限遠
        hit |= ~inside

        any_hit = hit.any(axis=1)
        first = np.where(any_hit, hit.argmax(axis=1), self.n_steps - 1)
        rng = self.ray_d[first]
        return np.where(any_hit, rng, np.inf)

    def _tick_scan(self):
        sec, nsec = self._stamp()
        s = LaserScan()
        s.header.stamp.sec, s.header.stamp.nanosec = sec, nsec
        s.header.frame_id = 'laser'
        s.angle_min = float(self.ray_a[0])
        s.angle_max = float(self.ray_a[-1])
        s.angle_increment = float(self.ray_a[1] - self.ray_a[0])
        s.time_increment = 0.0
        s.scan_time = 1.0 / SCAN_HZ
        s.range_min = 0.1
        s.range_max = RANGE_MAX
        s.ranges = self._raycast().astype(np.float32).tolist()
        self.scan_pub.publish(s)


def main():
    rclpy.init()
    node = SimStandin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
