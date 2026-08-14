/**
 * @file state_estimator_node.hpp
 * @brief StateEstimator 节点类声明
 *
 * Subscribes to:
 *   - /filtered_result  (fm_driver::msg::Result)
 *   - odom topic        (nav_msgs::msg::Odometry, configurable)
 *
 * Publishes:
 *   - /target_state     (state_estimator::msg::TargetState)
 *   - /system_state     (state_estimator::msg::SystemState)
 *   - /target_pose      (geometry_msgs::msg::PoseStamped)
 *   - /target_markers   (visualization_msgs::msg::MarkerArray)
 *
 * 坐标变换 (2D 机器人, 仅 yaw 旋转):
 *   p_odom = p_robot + R(θ) * p_rel
 *   v_odom = v_robot + R(θ) * v_rel + ω × (R(θ) * p_rel)
 */

#ifndef STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_
#define STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_

#include <cmath>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "fm_driver/msg/result.hpp"
#include "state_estimator/msg/target_state.hpp"
#include "state_estimator/msg/system_state.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// ---------------------------------------------------------------------------
// 状态估计节点
// ---------------------------------------------------------------------------
class StateEstimator : public rclcpp::Node
{
public:
  StateEstimator();

private:
  // ========================================================================
  //  类型别名
  // ========================================================================
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      fm_driver::msg::Result,
      nav_msgs::msg::Odometry>;

  using Sync = message_filters::Synchronizer<SyncPolicy>;

  // ========================================================================
  //  方法
  // ========================================================================
  void publish_visualization(
      const state_estimator::msg::TargetState &out,
      const double px, const double py, const double pz,
      const double vx, const double vy, const double vz);

  void sync_callback(
      const fm_driver::msg::Result::ConstSharedPtr &uwb,
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom);

  // ========================================================================
  //  成员
  // ========================================================================
  message_filters::Subscriber<fm_driver::msg::Result> sub_uwb_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom_;
  std::shared_ptr<Sync> sync_;

  rclcpp::Publisher<state_estimator::msg::TargetState>::SharedPtr pub_;
  rclcpp::Publisher<state_estimator::msg::SystemState>::SharedPtr pub_system_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  double sync_slop_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string uwb_frame_id_;
  std::string base_frame_id_;
  double tf_timeout_;
};

#endif  // STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_
