#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

// 航点类型：1=普通航点  2=抓取航点  3=投放航点
struct Target
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  int type = 1;
};

// 任务子状态机
enum class TaskPhase
{
  Idle,              // 普通巡航 / 任务航点未开始
  PickupAligning,    // 抓取航点：50cm 视觉对准
  PickupDescending,  // 抓取航点：已发 servo=01 + magnet=01，下降到 20cm
  PickupHolding,     // 抓取航点：在 20cm 悬停 1s
  PickupAscending,   // 抓取航点：已发 servo=00，上升到 50cm
  PickupObserving,   // 抓取航点：在 50cm 观察是否还能看到圆 1s
  DropArriving,      // 投放航点：飞到 (x,y,50cm)
  DropServoSent      // 投放航点：已发 servo=00，等待 1s 后发 magnet=00
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void addTarget(const Target & target);
  std::size_t currentIndex() const;
  std::size_t size() const;

private:
  void publishCurrent();
  void publishTarget(const Target & target, bool init_flag);
  Target getPublishedTarget(const Target & target) const;

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;
  bool hasFreshFineData(const rclcpp::Time & now_time) const;

  void monitorTimerCallback();
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void circleCenterCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void advanceToNextTarget();
  void publishVisualTakeoverState(bool active);
  void publishServoControl(uint8_t state);
  void publishElectromagnetControl(uint8_t state);
  void setPhase(TaskPhase phase, const rclcpp::Time & now_time);

  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_active_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr servo_control_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr electromagnet_control_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr circle_center_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::vector<Target> targets_;
  std::size_t current_idx_;

  bool has_height_;
  double current_height_cm_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;

  // 视觉对准参数
  double visual_align_pixel_threshold_;
  int visual_align_required_frames_;
  double visual_takeover_timeout_sec_;
  double fine_data_stale_timeout_sec_;

  // 抓取/投放任务参数
  double pickup_align_altitude_cm_;
  double pickup_grab_altitude_cm_;
  double pickup_hold_at_grab_sec_;
  double pickup_observe_sec_;
  int pickup_max_attempts_;
  double drop_altitude_cm_;
  double drop_servo_to_magnet_sec_;
  double circle_lost_window_sec_;

  // 视觉接管 / fine_data 状态
  bool visual_takeover_active_;
  bool has_fine_data_;
  int fine_error_x_px_;
  int fine_error_y_px_;
  rclcpp::Time last_fine_data_time_;

  // 圆心检测的新鲜度（用于抓取成功判定）
  bool has_circle_center_;
  rclcpp::Time last_circle_center_time_;

  bool mission_complete_sent_;

  int aligned_frame_count_;
  rclcpp::Time visual_takeover_start_time_;

  // 抓取/投放子状态机
  TaskPhase phase_;
  rclcpp::Time phase_start_time_;
  int pickup_attempts_;
};

class RouteTestNode : public rclcpp::Node
{
public:
  explicit RouteTestNode(
    const std::shared_ptr<RouteTargetPublisherNode> & route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::vector<Target> buildRoute() const;
  void loadRoute(const std::vector<Target> & route);

  std::shared_ptr<RouteTargetPublisherNode> route_node_;
};

}  // namespace activity_control_pkg
