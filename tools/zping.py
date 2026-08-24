#!/usr/bin/env python3
"""zenoh bridge probe: round-trip latency (ping/pong) and throughput (flood/sink).

ping 端與 pong 端各跑一個。RTT 全程用 ping 端的時鐘量（pong 只是把 header.stamp
原封不動抄回 /goal_pose），所以兩台機器不需要對時。

topic 選 /initialpose、/scan、/d455/color/image_raw、/goal_pose 是因為
zenoh allowlist 只放行這幾個；量測期間這些 topic 在兩端都沒有其他訂閱者。

expect-none 角色是白名單閘門的否定測試：搭 --topic 指定一個「不該過橋」的
topic，收到任何一則就算失敗。白名單寫錯時橋不會報錯，只會靜默放行或靜默阻擋，
沒有這個角色就分不出「擋住了」與「根本沒在跑」。

expect-none 單獨跑沒有意義：橋掛掉、網路斷、domain 設錯時它一樣是綠燈。必須與
同一時窗、同一組行程的正向 ping（白名單內 topic 且必須收到）配成一對，正向那半
才是「鏈路真的在跑」的證據。配對由驗收腳本負責，不在本檔內。
"""
import argparse, json, sys, time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, LaserScan

MODES = {
    "small": (PoseWithCovarianceStamped, "/initialpose"),
    "scan":  (LaserScan,                 "/scan"),
    "image": (Image,                     "/d455/color/image_raw"),
}
REPLY_TOPIC = "/goal_pose"


def resolve(mode, topic):
    cls, default_topic = MODES[mode]
    return cls, (topic or default_topic)


def make_payload(mode, stamp, points, width, height):
    cls, _ = MODES[mode]
    msg = cls()
    msg.header.stamp = stamp
    msg.header.frame_id = "zping"
    if mode == "scan":
        msg.angle_min, msg.angle_max = -3.14, 3.14
        msg.angle_increment = 6.28 / points
        msg.range_min, msg.range_max = 0.1, 30.0
        msg.ranges = [1.0] * points
    elif mode == "image":
        msg.width, msg.height = width, height
        msg.encoding = "rgb8"
        msg.step = width * 3
        msg.data = bytes(width * height * 3)
    return msg


class Flood(Node):
    """單向灌流量：以固定速率發，序號放在 header.frame_id 讓 sink 算掉包。"""

    def __init__(self, mode, rate, seconds, reliable, points, width, height, topic=""):
        super().__init__("zflood")
        cls, topic = resolve(mode, topic)
        self.mode = mode
        self.points, self.width, self.height = points, width, height
        self.pub = self.create_publisher(cls, topic, qos(reliable))
        self.seq = 0
        self.deadline = time.time() + seconds
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        if time.time() > self.deadline:
            return
        msg = make_payload(self.mode, self.get_clock().now().to_msg(),
                           self.points, self.width, self.height)
        msg.header.frame_id = str(self.seq)
        self.pub.publish(msg)
        self.seq += 1


class Sink(Node):
    """收端：算實際到達的訊息數、位元組數與序號缺口。"""

    def __init__(self, mode, reliable, topic=""):
        super().__init__("zsink")
        cls, topic = resolve(mode, topic)
        self.mode = mode
        self.sub = self.create_subscription(cls, topic, self.cb, qos(reliable))
        self.n, self.bytes = 0, 0
        self.first_t = None
        self.last_t = None
        self.seqs = []

    def cb(self, msg):
        now = time.time()
        if self.first_t is None:
            self.first_t = now
        self.last_t = now
        self.n += 1
        if self.mode == "image":
            self.bytes += len(msg.data)
        elif self.mode == "scan":
            self.bytes += 4 * len(msg.ranges)
        else:
            self.bytes += 736
        try:
            self.seqs.append(int(msg.header.frame_id))
        except ValueError:
            pass

    def report(self, label, offered_rate):
        dur = (self.last_t - self.first_t) if (self.n > 1) else 0.0
        out = {"label": label, "mode": self.mode, "offered_hz": offered_rate,
               "recv": self.n, "seconds": round(dur, 2)}
        if dur > 0:
            out["delivered_hz"] = round((self.n - 1) / dur, 1)
            out["delivered_MBps"] = round(self.bytes / dur / 1e6, 2)
        if self.seqs:
            sent_span = max(self.seqs) - min(self.seqs) + 1
            out["sent_span"] = sent_span
            out["loss_pct"] = round(100 * (1 - self.n / sent_span), 1)
        print("RESULT " + json.dumps(out))


class ExpectNone(Node):
    """否定測試：訂閱一個「不該送達」的 topic，收到任何一則即為失敗。

    刻意固定用 BEST_EFFORT 訂閱：BEST_EFFORT reader 同時相容 RELIABLE 與
    BEST_EFFORT writer，若跟著 --best-effort 走，QoS 不相容會讓訂閱根本配不上
    發布端，測試就會因為「沒配對」而假性通過。
    """

    def __init__(self, mode, topic=""):
        super().__init__("zexpect_none")
        cls, topic = resolve(mode, topic)
        self.mode_name, self.topic = mode, topic
        self.sub = self.create_subscription(cls, topic, self.cb, qos(False))
        self.n = 0

    def cb(self, msg):
        self.n += 1

    def report(self, label, seconds):
        ok = self.n == 0
        print("RESULT " + json.dumps({
            "label": label, "impl": "py", "role": "expect-none", "mode": self.mode_name,
            "topic": self.topic, "seconds": round(seconds, 2), "recv": self.n, "pass": ok}))
        print(f"PASS: no messages on {self.topic} in {seconds:.1f}s" if ok
              else f"FAIL: received {self.n} message(s) on {self.topic} in {seconds:.1f}s")
        return ok


def qos(reliable):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST, depth=10)


class Pong(Node):
    def __init__(self, mode, reliable, topic=""):
        super().__init__("zpong")
        cls, topic = resolve(mode, topic)
        self.pub = self.create_publisher(PoseStamped, REPLY_TOPIC, qos(True))
        self.sub = self.create_subscription(cls, topic, self.cb, qos(reliable))
        self.n = 0
        self.get_logger().info(f"pong: {topic} -> {REPLY_TOPIC}")

    def cb(self, msg):
        out = PoseStamped()
        out.header.stamp = msg.header.stamp      # 抄回原 stamp = ping 端時鐘
        out.header.frame_id = "zpong"
        self.pub.publish(out)
        self.n += 1
        if self.n % 20 == 0:
            self.get_logger().info(f"pong: {self.n} echoed")


class Ping(Node):
    def __init__(self, mode, count, rate, reliable, points, width, height, topic=""):
        super().__init__("zping")
        cls, topic = resolve(mode, topic)
        self.mode, self.count = mode, count
        self.points, self.width, self.height = points, width, height
        self.pub = self.create_publisher(cls, topic, qos(reliable))
        self.sub = self.create_subscription(PoseStamped, REPLY_TOPIC, self.cb, qos(True))
        self.rtts, self.sent = [], 0
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        if self.sent >= self.count:
            return
        now = self.get_clock().now()
        self.pub.publish(make_payload(self.mode, now.to_msg(), self.points, self.width, self.height))
        self.sent += 1

    def cb(self, msg):
        sent_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.rtts.append((self.get_clock().now().nanoseconds - sent_ns) / 1e6)


def pct(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["ping", "pong", "flood", "sink", "expect-none"], required=True)
    ap.add_argument("--mode", choices=list(MODES), default="small")
    ap.add_argument("--topic", default="", help="覆寫 --mode 的預設 topic（訊息型別仍由 --mode 決定）")
    ap.add_argument("--count", type=int, default=100)
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--points", type=int, default=1080)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--best-effort", action="store_true")
    ap.add_argument("--label", default="")
    ap.add_argument("--seconds", type=float, default=15.0)
    a = ap.parse_args()

    rclpy.init()
    reliable = not a.best_effort
    if a.role == "pong":
        node = Pong(a.mode, reliable, a.topic)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        rclpy.shutdown()
        return

    if a.role == "flood":
        node = Flood(a.mode, a.rate, a.seconds, reliable, a.points, a.width, a.height, a.topic)
        end = time.time() + a.seconds + 1.0
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        print("RESULT " + json.dumps({"label": a.label, "role": "flood", "mode": a.mode,
                                      "offered_hz": a.rate, "published": node.seq,
                                      "actual_pub_hz": round(node.seq / a.seconds, 1)}))
        rclpy.shutdown()
        return

    if a.role == "expect-none":
        node = ExpectNone(a.mode, a.topic)
        end = time.time() + a.seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        ok = node.report(a.label, a.seconds)
        rclpy.shutdown()
        sys.exit(0 if ok else 1)

    if a.role == "sink":
        node = Sink(a.mode, reliable, a.topic)
        end = time.time() + a.seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.report(a.label, a.rate)
        rclpy.shutdown()
        return

    node = Ping(a.mode, a.count, a.rate, reliable, a.points, a.width, a.height, a.topic)
    deadline = time.time() + a.count / a.rate + 8.0
    while rclpy.ok() and time.time() < deadline and len(node.rtts) < a.count:
        rclpy.spin_once(node, timeout_sec=0.05)
    r = node.rtts
    out = {"label": a.label, "mode": a.mode, "sent": node.sent, "recv": len(r),
           "loss_pct": round(100 * (1 - len(r) / max(1, node.sent)), 1)}
    if r:
        out.update({"rtt_min_ms": round(min(r), 3), "rtt_p50_ms": round(pct(r, 50), 3),
                    "rtt_p90_ms": round(pct(r, 90), 3), "rtt_p99_ms": round(pct(r, 99), 3),
                    "rtt_max_ms": round(max(r), 3),
                    "owd_p50_ms": round(pct(r, 50) / 2, 3)})
    print("RESULT " + json.dumps(out))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
