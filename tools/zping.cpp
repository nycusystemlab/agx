// zping.py 的 rclcpp 版本：同樣的 topic、同樣的量法（pong 把 header.stamp 原樣抄回），
// 用來把「rclpy 的成本」從「橋 + 網路的成本」裡拆出來。
// expect-none 角色與 Python 版同義：斷言指定 topic 收不到任何訊息，收到即失敗（exit 1）。
// 它單獨跑沒有意義——橋掛掉時一樣綠燈——必須與同一時窗的正向 ping 配成一對才有證據力。
#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
static const char *REPLY_TOPIC = "/goal_pose";

struct Opt {
  std::string role, mode = "small", label, topic;
  int count = 100, points = 1080, width = 640, height = 480;
  double rate = 10.0, seconds = 15.0;
};

static rclcpp::QoS qos() { return rclcpp::QoS(rclcpp::KeepLast(10)).reliable(); }

static std::string resolve(const Opt &o) {
  if (!o.topic.empty()) return o.topic;
  if (o.mode == "small") return "/initialpose";
  if (o.mode == "scan") return "/scan";
  return "/d455/color/image_raw";
}

class Pong : public rclcpp::Node {
public:
  Pong(const Opt &o) : Node("zpong_cpp") {
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(REPLY_TOPIC, qos());
    auto reply = [this](const std_msgs::msg::Header &h) {
      geometry_msgs::msg::PoseStamped out;
      out.header.stamp = h.stamp;              // 抄回原 stamp = ping 端時鐘
      out.header.frame_id = "zpong_cpp";
      pub_->publish(out);
    };
    const std::string topic = resolve(o);
    if (o.mode == "small")
      sub_small_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic, qos(),
          [reply](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) { reply(m->header); });
    else if (o.mode == "scan")
      sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
          topic, qos(), [reply](sensor_msgs::msg::LaserScan::SharedPtr m) { reply(m->header); });
    else
      sub_img_ = create_subscription<sensor_msgs::msg::Image>(
          topic, qos(),
          [reply](sensor_msgs::msg::Image::SharedPtr m) { reply(m->header); });
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_small_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
};

class Ping : public rclcpp::Node {
public:
  std::vector<double> rtts;
  int sent = 0;

  Ping(const Opt &o) : Node("zping_cpp"), o_(o) {
    const std::string topic = resolve(o);
    if (o.mode == "small") pub_small_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic, qos());
    else if (o.mode == "scan") pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(topic, qos());
    else pub_img_ = create_publisher<sensor_msgs::msg::Image>(topic, qos());

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        REPLY_TOPIC, qos(), [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          int64_t sent_ns = static_cast<int64_t>(m->header.stamp.sec) * 1000000000LL + m->header.stamp.nanosec;
          rtts.push_back((now().nanoseconds() - sent_ns) / 1e6);
        });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / o.rate), [this]() { tick(); });
  }

private:
  void tick() {
    if (sent >= o_.count) return;
    auto stamp = now();
    if (o_.mode == "small") {
      geometry_msgs::msg::PoseWithCovarianceStamped m;
      m.header.stamp = stamp; m.header.frame_id = "zping";
      pub_small_->publish(m);
    } else if (o_.mode == "scan") {
      sensor_msgs::msg::LaserScan m;
      m.header.stamp = stamp; m.header.frame_id = "zping";
      m.angle_min = -3.14f; m.angle_max = 3.14f;
      m.angle_increment = 6.28f / o_.points;
      m.range_min = 0.1f; m.range_max = 30.0f;
      m.ranges.assign(o_.points, 1.0f);
      pub_scan_->publish(m);
    } else {
      sensor_msgs::msg::Image m;
      m.header.stamp = stamp; m.header.frame_id = "zping";
      m.width = o_.width; m.height = o_.height; m.encoding = "rgb8"; m.step = o_.width * 3;
      m.data.assign(static_cast<size_t>(o_.width) * o_.height * 3, 0);
      pub_img_->publish(m);
    }
    sent++;
  }

  Opt o_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_small_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// 否定測試：訂閱一個「不該送達」的 topic，收到任何一則即為失敗。
// 刻意固定用 BEST_EFFORT 訂閱：BEST_EFFORT reader 同時相容 RELIABLE 與 BEST_EFFORT
// writer，若用 RELIABLE，QoS 不相容會讓訂閱根本配不上發布端，測試會假性通過。
class ExpectNone : public rclcpp::Node {
public:
  int n = 0;

  ExpectNone(const Opt &o) : Node("zexpect_none_cpp") {
    auto q = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const std::string topic = resolve(o);
    auto count = [this](const std_msgs::msg::Header &) { n++; };
    if (o.mode == "small")
      sub_small_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic, q,
          [count](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) { count(m->header); });
    else if (o.mode == "scan")
      sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
          topic, q, [count](sensor_msgs::msg::LaserScan::SharedPtr m) { count(m->header); });
    else
      sub_img_ = create_subscription<sensor_msgs::msg::Image>(
          topic, q, [count](sensor_msgs::msg::Image::SharedPtr m) { count(m->header); });
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_small_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
};

static double pct(std::vector<double> v, double p) {
  std::sort(v.begin(), v.end());
  size_t i = static_cast<size_t>(p / 100.0 * (v.size() - 1) + 0.5);
  return v[std::min(i, v.size() - 1)];
}

static const char *USAGE =
    "usage: zping --role {ping|pong|expect-none} [--mode {small|scan|image}] [--topic T]\n"
    "             [--count N] [--rate HZ] [--seconds S] [--points N] [--width W] [--height H]\n"
    "             [--label L]\n"
    "note: flood/sink 只有 zping.py 有\n";

// 未知的 --role 若沉默地落到 Ping，探針會變成發布端：small 模式會往 /initialpose
// 灌 100 則零位姿，那是白名單內的 topic，會過橋餵給車上的 AMCL。寧可直接拒絕啟動。
static bool parse(int argc, char **argv, Opt &o) {
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--ros-args") break;
    if (i + 1 >= argc) {
      fprintf(stderr, "zping: %s 缺少值\n%s", a.c_str(), USAGE);
      return false;
    }
    std::string v = argv[++i];
    if (a == "--role") o.role = v;
    else if (a == "--mode") o.mode = v;
    else if (a == "--label") o.label = v;
    else if (a == "--topic") o.topic = v;
    else if (a == "--count") o.count = std::stoi(v);
    else if (a == "--rate") o.rate = std::stod(v);
    else if (a == "--points") o.points = std::stoi(v);
    else if (a == "--width") o.width = std::stoi(v);
    else if (a == "--height") o.height = std::stoi(v);
    else if (a == "--seconds") o.seconds = std::stod(v);
    else {
      fprintf(stderr, "zping: 不認得的參數 %s\n%s", a.c_str(), USAGE);
      return false;
    }
  }
  if (o.role != "ping" && o.role != "pong" && o.role != "expect-none") {
    fprintf(stderr, "zping: 不認得的 --role '%s'\n%s", o.role.c_str(), USAGE);
    return false;
  }
  if (o.mode != "small" && o.mode != "scan" && o.mode != "image") {
    fprintf(stderr, "zping: 不認得的 --mode '%s'\n%s", o.mode.c_str(), USAGE);
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  Opt o;
  if (!parse(argc, argv, o)) { rclcpp::shutdown(); return 2; }

  if (o.role == "pong") {
    rclcpp::spin(std::make_shared<Pong>(o));
    rclcpp::shutdown();
    return 0;
  }

  if (o.role == "expect-none") {
    auto node = std::make_shared<ExpectNone>(o);
    const std::string topic = resolve(o);
    auto end = std::chrono::steady_clock::now() +
               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(o.seconds));
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) exec.spin_once(50ms);
    bool ok = node->n == 0;
    printf("RESULT {\"label\": \"%s\", \"impl\": \"cpp\", \"role\": \"expect-none\", "
           "\"mode\": \"%s\", \"topic\": \"%s\", \"seconds\": %.2f, \"recv\": %d, "
           "\"pass\": %s}\n",
           o.label.c_str(), o.mode.c_str(), topic.c_str(), o.seconds, node->n,
           ok ? "true" : "false");
    if (ok) printf("PASS: no messages on %s in %.1fs\n", topic.c_str(), o.seconds);
    else printf("FAIL: received %d message(s) on %s in %.1fs\n", node->n, topic.c_str(), o.seconds);
    rclcpp::shutdown();
    return ok ? 0 : 1;
  }

  auto node = std::make_shared<Ping>(o);
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<double>(o.count / o.rate + 8.0));
  // 必須是帶 timeout 的 spin_once：無事可做時 spin_some 會立刻返回，這個迴圈就把一個
  // 核心跑滿，而 CPU 競爭會灌水這支探針自己要量的 RTT（Python 版同樣用 0.05s）。
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
         static_cast<int>(node->rtts.size()) < o.count)
    exec.spin_once(50ms);

  auto &r = node->rtts;
  printf("RESULT {\"label\": \"%s\", \"impl\": \"cpp\", \"mode\": \"%s\", \"sent\": %d, \"recv\": %zu",
         o.label.c_str(), o.mode.c_str(), node->sent, r.size());
  if (!r.empty())
    printf(", \"rtt_min_ms\": %.3f, \"rtt_p50_ms\": %.3f, \"rtt_p90_ms\": %.3f, \"rtt_p99_ms\": %.3f, \"rtt_max_ms\": %.3f",
           *std::min_element(r.begin(), r.end()), pct(r, 50), pct(r, 90), pct(r, 99),
           *std::max_element(r.begin(), r.end()));
  printf("}\n");
  rclcpp::shutdown();
  return 0;
}
