/**
 * @file state_estimator_node.cpp
 * @brief Transform filtered UWB target state to odometry frame.
 *
 * Subscribes to:
 *   - /filtered_result  (fm_driver::msg::Result)
 *   - odom topic        (nav_msgs::msg::Odometry, configurable)
 *
 * Publishes:
 *   - /target_state     (state_estimator::msg::TargetState)
 *   - /system_state     (state_estimator::msg::SystemState)
 *
 * 坐标变换 (2D 机器人, 仅 yaw 旋转):
 *   p_odom = p_robot + R(θ) * p_rel
 *   v_odom = v_robot + R(θ) * v_rel + ω × (R(θ) * p_rel)
 */

#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "fm_driver/msg/result.hpp"
#include "state_estimator/msg/target_state.hpp"
#include "state_estimator/msg/system_state.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ---------------------------------------------------------------------------
// 状态估计节点
// ---------------------------------------------------------------------------
class StateEstimator : public rclcpp::Node
{
public:
  StateEstimator()
      : Node("state_estimator")
  {
    // ----- 参数声明 -----
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("output_topic", "/target_state");
    this->declare_parameter("sync_slop", 0.05); // 近似时间对齐容差 (s)
    this->declare_parameter("sync_queue_size", 10);

    // ----- 读取参数 -----
    const std::string odom_topic = this->get_parameter("odom_topic").as_string();
    const std::string output_topic = this->get_parameter("output_topic").as_string();
    sync_slop_ = this->get_parameter("sync_slop").as_double();
    const int queue_size = this->get_parameter("sync_queue_size").as_int();

    // ----- 发布器 -----
    pub_ = this->create_publisher<state_estimator::msg::TargetState>(output_topic, 10);
    pub_system_ = this->create_publisher<state_estimator::msg::SystemState>("/system_state", 10);

    // ----- 同步订阅 (message_filters ApproximateTime) -----
    sub_uwb_.subscribe(this, "/filtered_result",
                       rmw_qos_profile_sensor_data);
    sub_odom_.subscribe(this, odom_topic,
                        rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<Sync>(SyncPolicy(queue_size),
                                   sub_uwb_,
                                   sub_odom_);
    sync_->setAgePenalty(sync_slop_); // 越旧越被优先丢弃
    sync_->registerCallback(
        std::bind(&StateEstimator::sync_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "StateEstimator started | odom=%s output=%s sync_slop=%.3fs",
                odom_topic.c_str(), output_topic.c_str(), sync_slop_);
  }

private:
  // ========================================================================
  //  同步回调
  // ========================================================================
  void sync_callback(
      const fm_driver::msg::Result::ConstSharedPtr &uwb,
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
  {
    RCLCPP_INFO(this->get_logger(), "Received sync messages.");
    // --- 从里程计提取机器人 2D 位姿 ---
    double yaw = 0.0;
    {
      tf2::Quaternion q;
      tf2::fromMsg(odom->pose.pose.orientation, q);
      yaw = q.getAngleShortestPath(); // wrapper 不一定有 getYaw，用 Matrix3x3 安全
      // 实际上 q.getAngleShortestPath() 返回的是旋转角，不是 yaw。需要修正——
      // 用 Matrix3x3 提取 RPY
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    const double cos_y = std::cos(yaw);
    const double sin_y = std::sin(yaw);

    const double rx = odom->pose.pose.position.x;
    const double ry = odom->pose.pose.position.y;
    const double rz = odom->pose.pose.position.z;

    const double vx = odom->twist.twist.linear.x;
    const double vy = odom->twist.twist.linear.y;
    const double vz = odom->twist.twist.linear.z;

    const double wz = odom->twist.twist.angular.z; // 2D: 仅保留 ω_z

    // --- 旋转相对位置到 odom 系 ---
    const double px_rel = uwb->pos[0];
    const double py_rel = uwb->pos[1];
    const double pz_rel = uwb->pos[2];

    const double rpx = cos_y * px_rel - sin_y * py_rel;
    const double rpy = sin_y * px_rel + cos_y * py_rel;
    const double rpz = pz_rel;

    // 位置: p_odom = p_robot + R * p_rel
    const double px_odom = rx + rpx;
    const double py_odom = ry + rpy;
    const double pz_odom = rz + rpz;

    // --- 旋转相对速度到 odom 系 ---
    const double vx_rel = uwb->vel[0];
    const double vy_rel = uwb->vel[1];
    const double vz_rel = uwb->vel[2];

    const double rvx = cos_y * vx_rel - sin_y * vy_rel;
    const double rvy = sin_y * vx_rel + cos_y * vy_rel;
    const double rvz = vz_rel;

    // 牵连速度: ω × (R * p_rel) = (-wz * rpy,  wz * rpx,  0)
    const double tx = -wz * rpy;
    const double ty = wz * rpx;

    // 速度: v_odom = v_robot + R * v_rel + ω × (R * p_rel)
    const double vx_odom = vx + rvx + tx;
    const double vy_odom = vy + rvy + ty;
    const double vz_odom = vz + rvz; // ω×r 在 z 分量为 0

    // --- 噪声变换: R * σ ---
    const double npx = cos_y * uwb->pos_noise[0] - sin_y * uwb->pos_noise[1];
    const double npy = sin_y * uwb->pos_noise[0] + cos_y * uwb->pos_noise[1];
    const double npz = uwb->pos_noise[2];

    const double nvx = cos_y * uwb->vel_noise[0] - sin_y * uwb->vel_noise[1];
    const double nvy = sin_y * uwb->vel_noise[0] + cos_y * uwb->vel_noise[1];
    const double nvz = uwb->vel_noise[2];

    // --- 极坐标 (基于原始相对位置) ---
    const double dis = std::sqrt(px_rel * px_rel + py_rel * py_rel + pz_rel * pz_rel);
    const double azimuth = std::atan2(py_rel, px_rel);
    const double elevation = std::atan2(pz_rel, std::sqrt(px_rel * px_rel + py_rel * py_rel));

    // --- 组装输出消息 ---
    state_estimator::msg::TargetState out;
    out.header = uwb->header;
    out.header.stamp = this->now();
    out.header.frame_id = "odom";

    out.pos[0] = px_odom;
    out.pos[1] = py_odom;
    out.pos[2] = pz_odom;
    out.vel[0] = vx_odom;
    out.vel[1] = vy_odom;
    out.vel[2] = vz_odom;

    out.pos_noise[0] = npx;
    out.pos_noise[1] = npy;
    out.pos_noise[2] = npz;
    out.vel_noise[0] = nvx;
    out.vel_noise[1] = nvy;
    out.vel_noise[2] = nvz;

    out.dis = dis;
    out.azimuth = azimuth;
    out.elevation = elevation;

    // --- 发布 ---
    pub_->publish(out);

    // --- 系统状态 ---
    state_estimator::msg::SystemState sys_out;
    sys_out.header = uwb->header;
    sys_out.header.stamp = this->now();
    sys_out.header.frame_id = "odom";

    double dx = rx - px_odom;
    double dy = ry - py_odom;
    double cross = dx * vy_odom - dy * vx_odom;
    double dot = dx * vx_odom + dy * vy_odom;
    double alpha = std::atan2(cross, dot);

    sys_out.alpha = alpha;

    cross = vx * vy_odom - vy * vx_odom;
    dot = vx * vx_odom + vy * vy_odom;
    double beta = std::atan2(cross, dot);

    sys_out.beta = beta;
    sys_out.rho = dis;

    // --- 发布 ---
    pub_system_->publish(sys_out);
  }

  // ========================================================================
  //  类型别名
  // ========================================================================
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      fm_driver::msg::Result,
      nav_msgs::msg::Odometry>;

  using Sync = message_filters::Synchronizer<SyncPolicy>;

  // ========================================================================
  //  成员
  // ========================================================================
  message_filters::Subscriber<fm_driver::msg::Result> sub_uwb_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom_;
  std::shared_ptr<Sync> sync_;

  rclcpp::Publisher<state_estimator::msg::TargetState>::SharedPtr pub_;
  rclcpp::Publisher<state_estimator::msg::SystemState>::SharedPtr pub_system_;

  double sync_slop_;
};

// ===========================================================================
//  main
// ===========================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}
