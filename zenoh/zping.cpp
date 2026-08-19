// zping.py 的 rclcpp 版本：同樣的 topic、同樣的量法（pong 把 header.stamp 原樣抄回），
// 用來把「rclpy 的成本」從「橋 + 網路的成本」裡拆出來。
#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
static const char *REPLY_TOPIC = "/goal_pose";

struct Opt {
  std::string role, mode = "small", label;
  int count = 100, points = 1080, width = 640, height = 480;
  double rate = 10.0, seconds = 15.0;
};

static rclcpp::QoS qos() { return rclcpp::QoS(rclcpp::KeepLast(10)).reliable(); }

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
    if (o.mode == "small")
      sub_small_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos(),
          [reply](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) { reply(m->header); });
    else if (o.mode == "scan")
      sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", qos(), [reply](sensor_msgs::msg::LaserScan::SharedPtr m) { reply(m->header); });
    else
      sub_img_ = create_subscription<sensor_msgs::msg::Image>(
          "/d455/color/image_raw", qos(),
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
    if (o.mode == "small") pub_small_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", qos());
    else if (o.mode == "scan") pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", qos());
    else pub_img_ = create_publisher<sensor_msgs::msg::Image>("/d455/color/image_raw", qos());

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

static double pct(std::vector<double> v, double p) {
  std::sort(v.begin(), v.end());
  size_t i = static_cast<size_t>(p / 100.0 * (v.size() - 1) + 0.5);
  return v[std::min(i, v.size() - 1)];
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  Opt o;
  for (int i = 1; i < argc - 1; i++) {
    std::string a = argv[i], v = argv[i + 1];
    if (a == "--role") o.role = v;
    else if (a == "--mode") o.mode = v;
    else if (a == "--label") o.label = v;
    else if (a == "--count") o.count = std::stoi(v);
    else if (a == "--rate") o.rate = std::stod(v);
    else if (a == "--points") o.points = std::stoi(v);
    else if (a == "--width") o.width = std::stoi(v);
    else if (a == "--height") o.height = std::stoi(v);
    else if (a == "--seconds") o.seconds = std::stod(v);
  }

  if (o.role == "pong") {
    rclcpp::spin(std::make_shared<Pong>(o));
    rclcpp::shutdown();
    return 0;
  }

  auto node = std::make_shared<Ping>(o);
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<double>(o.count / o.rate + 8.0));
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
         static_cast<int>(node->rtts.size()) < o.count)
    rclcpp::spin_some(node);

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
