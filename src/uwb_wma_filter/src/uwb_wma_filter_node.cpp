/**
 * @file uwb_wma_filter_node.cpp
 * @brief Weighted Moving Average filter for UWB robot-following system.
 *
 * Filters the FM-Driver /Result stream (position + velocity) using a
 * noise-weighted and time-decayed sliding window. Publishes to
 * /filtered_result with the same message type.
 *
 * 每维独立权重:
 *   w_i^d = (1 / max(σ_i^d², ε_d²)) * exp(-α * Δt_i)
 *
 * 滤波值 (d = 0,1,2):
 *   x̂[d] = Σ(w_i^d * x_i[d]) / Σ(w_i^d)
 *
 * 输出噪声 (d = 0,1,2):
 *   σ̂_out[d] = sqrt(Σ((w_i^d)² * (σ_i^d)²)) / Σ(w_i^d)
 */

#include <deque>
#include <cmath>
#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "fm_driver/msg/result.hpp"

// ---------------------------------------------------------------------------
// 内部结构：带时间戳的消息拷贝
// ---------------------------------------------------------------------------
struct StampedResult
{
  rclcpp::Time stamp;
  fm_driver::msg::Result msg;
};

// ---------------------------------------------------------------------------
// 滤波器节点
// ---------------------------------------------------------------------------
class UwbWmaFilter : public rclcpp::Node
{
public:
  UwbWmaFilter()
      : Node("uwb_wma_filter")
  {
    // ----- 声明 ROS 参数 -----
    this->declare_parameter("window_size", 0.3);
    this->declare_parameter("min_samples", 5);
    this->declare_parameter("noise_floor_pos", 0.01); // m
    this->declare_parameter("noise_floor_vel", 0.01); // m/s
    this->declare_parameter("decay_rate", 5.0);       // 1/s
    this->declare_parameter("input_topic", "/fm_driver/result");
    this->declare_parameter("output_topic", "/filtered_result");

    // ----- 读取参数 -----
    window_size_ = this->get_parameter("window_size").as_double();
    min_samples_ = this->get_parameter("min_samples").as_int();
    noise_floor_pos_ = this->get_parameter("noise_floor_pos").as_double();
    noise_floor_vel_ = this->get_parameter("noise_floor_vel").as_double();
    decay_rate_ = this->get_parameter("decay_rate").as_double();
    const std::string input_topic = this->get_parameter("input_topic").as_string();
    const std::string output_topic = this->get_parameter("output_topic").as_string();

    // 噪声下限平方（方差下限），避免 σ→0 时 1/σ²→∞
    noise_floor_pos_var_ = noise_floor_pos_ * noise_floor_pos_;
    noise_floor_vel_var_ = noise_floor_vel_ * noise_floor_vel_;

    // ----- 发布 & 订阅 -----
    sub_ = this->create_subscription<fm_driver::msg::Result>(
        input_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&UwbWmaFilter::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<fm_driver::msg::Result>(output_topic, 10);

    RCLCPP_INFO(this->get_logger(),
                "UWB WMA Filter started | window=%.2fs min_samples=%d "
                "noise_floor(pos=%.3fm vel=%.3fm/s) decay_rate=%.2f 1/s",
                window_size_, min_samples_,
                noise_floor_pos_, noise_floor_vel_, decay_rate_);
  }

private:
  // ========================================================================
  //  回调：每收到一条 Result 消息触发
  // ========================================================================
  void callback(const fm_driver::msg::Result::SharedPtr raw)
  {
    // --- 丢包 / 乱序检测 (基于 cnt 序列号) ---
    if (has_prev_ && raw->cnt != static_cast<uint8_t>((prev_cnt_ + 1) % 256))
    {
      int gap = static_cast<int>(raw->cnt) - static_cast<int>(prev_cnt_);
      if (gap < 0)
        gap += 256; // 滚动回绕
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Possible packet loss or disorder: prev_cnt=%u curr_cnt=%u gap=%d",
                           prev_cnt_, raw->cnt, gap);
    }
    has_prev_ = true;
    prev_cnt_ = raw->cnt;

    // --- 插入缓冲区（按 header.stamp 保持有序）---
    StampedResult entry;
    entry.stamp = raw->header.stamp;
    entry.msg = *raw;
    insert_sorted(std::move(entry));

    // --- 修剪过期数据（以最新消息时间为基准）---
    prune_expired(raw->header.stamp);

    // --- 检查最小样本数 ---
    const int n = static_cast<int>(buffer_.size());
    if (n < min_samples_)
    {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Waiting for %d samples (currently %d)", min_samples_, n);
      return;
    }

    // --- 加权移动平均 ---
    auto filtered = compute_wma(raw->header);

    // --- 发布 ---
    pub_->publish(filtered);
  }

  // ========================================================================
  //  加权移动平均 —— 每维独立权重
  // ========================================================================
  fm_driver::msg::Result compute_wma(const std_msgs::msg::Header &src_header)
  {
    fm_driver::msg::Result out;
    out.header = src_header;
    out.header.stamp = this->now();
    out.local_time = this->now().nanoseconds();
    out.cnt = prev_cnt_;

    const double t_newest = rclcpp::Time(src_header.stamp).seconds();
    const size_t n = buffer_.size();

    // 累加器：每维独立
    double total_w_pos[3] = {0.0, 0.0, 0.0};
    double total_w_vel[3] = {0.0, 0.0, 0.0};

    double sum_wx_pos[3] = {0.0, 0.0, 0.0}; // Σ w_i^d * pos_i[d]
    double sum_wx_vel[3] = {0.0, 0.0, 0.0}; // Σ w_i^d * vel_i[d]

    double sum_w2s2_pos[3] = {0.0, 0.0, 0.0}; // Σ (w_i^d)² * (σ_i^d)²
    double sum_w2s2_vel[3] = {0.0, 0.0, 0.0};

    for (size_t i = 0; i < n; ++i)
    {
      const double age = t_newest - buffer_[i].stamp.seconds();
      const double time_decay = std::exp(-decay_rate_ * age);

      const auto &raw = buffer_[i].msg;

      for (int d = 0; d < 3; ++d)
      {
        // ---- 位置维 ----
        double sig_pos = std::fabs(raw.pos_noise[d]);
        double var_pos = std::max(sig_pos * sig_pos, noise_floor_pos_var_);
        double w_pos = (1.0 / var_pos) * time_decay;

        total_w_pos[d] += w_pos;
        sum_wx_pos[d] += w_pos * raw.pos[d];
        sum_w2s2_pos[d] += w_pos * w_pos * sig_pos * sig_pos;

        // ---- 速度维 ----
        double sig_vel = std::fabs(raw.vel_noise[d]);
        double var_vel = std::max(sig_vel * sig_vel, noise_floor_vel_var_);
        double w_vel = (1.0 / var_vel) * time_decay;

        total_w_vel[d] += w_vel;
        sum_wx_vel[d] += w_vel * raw.vel[d];
        sum_w2s2_vel[d] += w_vel * w_vel * sig_vel * sig_vel;
      }
    }

    // 归一化
    for (int d = 0; d < 3; ++d)
    {
      out.pos[d] = sum_wx_pos[d] / total_w_pos[d];
      out.vel[d] = sum_wx_vel[d] / total_w_vel[d];
      out.pos_noise[d] = std::sqrt(sum_w2s2_pos[d]) / total_w_pos[d];
      out.vel_noise[d] = std::sqrt(sum_w2s2_vel[d]) / total_w_vel[d];
    }

    return out;
  }

  // ========================================================================
  //  按 header.stamp 有序插入
  // ========================================================================
  void insert_sorted(StampedResult entry)
  {
    auto it = buffer_.begin();
    while (it != buffer_.end() && it->stamp < entry.stamp)
    {
      ++it;
    }
    buffer_.insert(it, std::move(entry));
  }

  // ========================================================================
  //  修剪超出窗口的旧样本
  // ========================================================================
  void prune_expired(const rclcpp::Time &newest)
  {
    const double t_new = newest.seconds();
    while (!buffer_.empty())
    {
      double age = t_new - buffer_.front().stamp.seconds();
      if (age > window_size_)
      {
        buffer_.pop_front();
      }
      else
      {
        break;
      }
    }
  }

  // ========================================================================
  //  成员变量
  // ========================================================================
  rclcpp::Subscription<fm_driver::msg::Result>::SharedPtr sub_;
  rclcpp::Publisher<fm_driver::msg::Result>::SharedPtr pub_;

  std::deque<StampedResult> buffer_;

  double window_size_;
  int min_samples_;
  double noise_floor_pos_;
  double noise_floor_vel_;
  double noise_floor_pos_var_;
  double noise_floor_vel_var_;
  double decay_rate_;

  bool has_prev_{false};
  uint8_t prev_cnt_{0};
};

// ===========================================================================
//  main
// ===========================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UwbWmaFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
