/**
 * @file state_estimator_node.cpp
 * @brief StateEstimator 节点类实现
 */

#include "state_estimator/state_estimator_node.hpp"

// ---------------------------------------------------------------------------
// 构造函数
// ---------------------------------------------------------------------------
StateEstimator::StateEstimator()
    : Node("state_estimator")
{
  // ----- 参数声明 -----
  this->declare_parameter("odom_topic", "/odom");
  this->declare_parameter("sync_slop", 0.05); // 近似时间对齐容差 (s)
  this->declare_parameter("sync_queue_size", 10);
  this->declare_parameter("uwb_frame_id", "fm_anchor_link");
  this->declare_parameter("base_frame_id", "BASE_LINK");
  this->declare_parameter("tf_timeout", 0.1); // TF 查询超时 (s)

  // ----- 读取参数 -----
  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  sync_slop_ = this->get_parameter("sync_slop").as_double();
  const int queue_size = this->get_parameter("sync_queue_size").as_int();
  uwb_frame_id_ = this->get_parameter("uwb_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  tf_timeout_ = this->get_parameter("tf_timeout").as_double();

  // ----- TF Buffer / Listener -----
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ----- 发布器 -----
  pub_ = this->create_publisher<state_estimator::msg::TargetState>("target_state", 10);
  pub_system_ = this->create_publisher<state_estimator::msg::SystemState>("/system_state", 10);
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("target_markers", 10);

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
              "StateEstimator started | odom=%s output=target_state sync_slop=%.3fs uwb_frame=%s base_frame=%s tf_timeout=%.3fs",
              odom_topic.c_str(), sync_slop_,
              uwb_frame_id_.c_str(), base_frame_id_.c_str(), tf_timeout_);
}

// ---------------------------------------------------------------------------
// publish_visualization: 组装并发布 RViz 可视化消息
// ---------------------------------------------------------------------------
void StateEstimator::publish_visualization(
    const state_estimator::msg::TargetState &out,
    const double px, const double py, const double pz,
    const double vx, const double vy, const double vz)
{
  const auto stamp = out.header.stamp;
  const std::string frame_id = "odom";

  // ---- PoseStamped (位置) ----
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id;

    pose.pose.position.x = px;
    pose.pose.position.y = py;
    pose.pose.position.z = pz;

    // 根据速度方向计算朝向 (yaw)
    const double yaw = std::atan2(vy, vx);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    pub_pose_->publish(pose);
  }

  // ---- MarkerArray (位置球体 + 速度箭头) ----
  visualization_msgs::msg::MarkerArray markers;

  // Marker 1: 位置球体 (SPHERE)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.ns = "target";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = px;
    m.pose.position.y = py;
    m.pose.position.z = pz;
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.3;  // 直径 30cm
    m.scale.y = 0.3;
    m.scale.z = 0.3;

    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.8;

    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    markers.markers.push_back(m);
  }

  // Marker 2: 速度箭头 (ARROW)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.ns = "target";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    // 设置起点 (目标位置) 和终点 (位置 + 速度向量)
    geometry_msgs::msg::Point start;
    start.x = px;
    start.y = py;
    start.z = pz;

    geometry_msgs::msg::Point end;
    const double vel_scale = 1.0; // 速度向量缩放倍数 (1m/s -> 1m 箭头长度)
    end.x = px + vx * vel_scale;
    end.y = py + vy * vel_scale;
    end.z = pz + vz * vel_scale;

    m.points.push_back(start);
    m.points.push_back(end);

    m.scale.x = 0.05;  // 箭杆直径
    m.scale.y = 0.10;  // 箭头直径
    m.scale.z = 0.15;  // 箭头长度

    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 1.0;

    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    markers.markers.push_back(m);
  }

  pub_markers_->publish(markers);
}

// ---------------------------------------------------------------------------
// sync_callback: UWB + Odom 同步回调
// ---------------------------------------------------------------------------
void StateEstimator::sync_callback(
    const fm_driver::msg::Result::ConstSharedPtr &uwb,
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
{
  RCLCPP_INFO(this->get_logger(), "Received sync messages.");

  // ======================================================================
  //  Step 1: 查询 TF 变换 (uwb_frame_id -> base_frame_id)
  //          即 fm_anchor_link -> BASE_LINK
  //  注: 该 TF 为静态外参，直接查询最新值 (TimePointZero)，无需时间对齐
  // ======================================================================
  geometry_msgs::msg::TransformStamped tf_uwb_to_base;
  try {
    if (!tf_buffer_->canTransform(base_frame_id_, uwb_frame_id_,
                                  tf2::TimePointZero,
                                  tf2::durationFromSec(tf_timeout_))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "TF unavailable (static): %s -> %s (timeout=%.3fs). "
          "Skipping this UWB sample. Is static_transform_publisher running?",
          uwb_frame_id_.c_str(), base_frame_id_.c_str(), tf_timeout_);
      return;
    }
    tf_uwb_to_base = tf_buffer_->lookupTransform(
        base_frame_id_, uwb_frame_id_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed (%s -> %s, static): %s. Skipping this UWB sample.",
        uwb_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
    return;
  }

  // 提取 fm_anchor_link -> BASE_LINK 的旋转矩阵和平移向量
  tf2::Quaternion q_uwb_to_base;
  tf2::fromMsg(tf_uwb_to_base.transform.rotation, q_uwb_to_base);
  tf2::Matrix3x3 R_uwb_to_base(q_uwb_to_base);
  const double tx_uwb = tf_uwb_to_base.transform.translation.x;
  const double ty_uwb = tf_uwb_to_base.transform.translation.y;
  const double tz_uwb = tf_uwb_to_base.transform.translation.z;

  RCLCPP_DEBUG(this->get_logger(),
      "TF %s->%s: t=(%.3f,%.3f,%.3f) q=(%.3f,%.3f,%.3f,%.3f)",
      uwb_frame_id_.c_str(), base_frame_id_.c_str(),
      tx_uwb, ty_uwb, tz_uwb,
      q_uwb_to_base.x(), q_uwb_to_base.y(), q_uwb_to_base.z(), q_uwb_to_base.w());

  // ======================================================================
  //  Step 2: 将 UWB 数据 (pos/vel/noise) 从 fm_anchor_link 变换到 BASE_LINK
  // ======================================================================
  // 位置: 需要平移 + 旋转
  const double px_anchor = uwb->pos[0];
  const double py_anchor = uwb->pos[1];
  const double pz_anchor = uwb->pos[2];
  tf2::Vector3 p_anchor(px_anchor, py_anchor, pz_anchor);
  tf2::Vector3 p_base = R_uwb_to_base * p_anchor + tf2::Vector3(tx_uwb, ty_uwb, tz_uwb);
  const double px_rel = p_base.x();
  const double py_rel = p_base.y();
  const double pz_rel = p_base.z();

  // 速度: 仅旋转 (自由向量，无平移)
  const double vx_anchor = uwb->vel[0];
  const double vy_anchor = uwb->vel[1];
  const double vz_anchor = uwb->vel[2];
  tf2::Vector3 v_anchor(vx_anchor, vy_anchor, vz_anchor);
  tf2::Vector3 v_base = R_uwb_to_base * v_anchor;
  const double vx_rel = v_base.x();
  const double vy_rel = v_base.y();
  const double vz_rel = v_base.z();

  // 噪声: 仅旋转 (与速度向量同理)
  tf2::Vector3 np_anchor(uwb->pos_noise[0], uwb->pos_noise[1], uwb->pos_noise[2]);
  tf2::Vector3 np_base = R_uwb_to_base * np_anchor;
  tf2::Vector3 nv_anchor(uwb->vel_noise[0], uwb->vel_noise[1], uwb->vel_noise[2]);
  tf2::Vector3 nv_base = R_uwb_to_base * nv_anchor;

  // ======================================================================
  //  Step 3: 从里程计提取机器人 2D 位姿 (BASE_LINK -> odom)
  // ======================================================================
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
  const double rpx = cos_y * px_rel - sin_y * py_rel;
  const double rpy = sin_y * px_rel + cos_y * py_rel;
  const double rpz = pz_rel;

  // 位置: p_odom = p_robot + R * p_rel
  const double px_odom = rx + rpx;
  const double py_odom = ry + rpy;
  const double pz_odom = rz + rpz;

  // --- 旋转相对速度到 odom 系 ---
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
  const double npx = cos_y * np_base.x() - sin_y * np_base.y();
  const double npy = sin_y * np_base.x() + cos_y * np_base.y();
  const double npz = np_base.z();

  const double nvx = cos_y * nv_base.x() - sin_y * nv_base.y();
  const double nvy = sin_y * nv_base.x() + cos_y * nv_base.y();
  const double nvz = nv_base.z();

  // --- 极坐标 (基于变换到 BASE_LINK 后的相对位置) ---
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

  // ======================================================================
  //  Step 5: 组装并发布 RViz 可视化消息
  // ======================================================================
  publish_visualization(out, px_odom, py_odom, pz_odom,
                        vx_odom, vy_odom, vz_odom);

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
