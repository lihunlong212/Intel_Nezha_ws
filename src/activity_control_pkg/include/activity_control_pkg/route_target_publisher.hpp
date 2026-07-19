#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
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
  uint8_t route_stage = 1;
};

// 任务子状态机
enum class TaskPhase
{
  Idle,              // 普通巡航 / 任务航点未开始
  SearchApproaching, // 搜索：保持当前高度，用视觉 XY 粗靠近目标
  PickupAligning,    // 抓取：在 50cm 视觉对准（高度 + XY）
  PickupDescending,  // 抓取：magnet=01 + servo=01 已发，视觉微调 XY 并下降到抓取高度
  PickupHolding,     // 抓取：在抓取高度悬停一段时间，让磁铁吸住货物
  PickupAscending,   // 抓取：servo=00 已发，上升回 50cm（电磁铁仍通电吸住货物）
  PickupObserving,   // 抓取：观察目标 AprilTag 是否仍可见来判定成败
  DropArriving,      // 投放：到点后收舵机，等待后下降到对准高度
  DropAligning,      // 投放：在对准高度使用 AprilTag 视觉对准
  DropServoSettling, // 投放：舵机下放后在对准高度等待
  DropDescending,    // 投放：锁定 XY 速度后下降到投放高度
  DropActing         // 投放：立即断磁铁，等待后收起舵机
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
  void routeStageCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void advanceToNextTarget();
  void enterFailureHold(
    const rclcpp::Time & now_time, const char * reason, bool drop_failure);
  void startDropFailureHold(const rclcpp::Time & now_time);
  void startSearchFailureHold(const rclcpp::Time & now_time);
  void startPickupFailureHold(const rclcpp::Time & now_time);
  void insertPostPickupClimbTarget(double x_cm, double y_cm, double yaw_deg);
  void removePendingSearchTargetsAfterCurrent();
  bool hasPendingSearchTargetsAfterCurrent() const;
  void resetFineDataState();
  void publishIdleVisionModeForCurrentTarget();
  void publishVisualTakeoverState(bool active);
  void publishXyVelocityHoldState(bool active);
  void publishFailureHoldState(bool active);
  void publishVisionTargetMode(uint8_t mode);
  void publishServoControl(uint8_t state);
  void publishElectromagnetControl(uint8_t state);
  void publishRouteHoldState();
  bool isCurrentTargetStageAllowed() const;
  void setPhase(TaskPhase phase, const rclcpp::Time & now_time);

  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr xy_velocity_hold_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr failure_hold_active_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vision_target_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr servo_control_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr electromagnet_control_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pickup_done_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pickup_failed_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr drop_done_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr drop_failed_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr route_stage_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::vector<Target> targets_;
  std::size_t current_idx_;
  uint8_t allowed_route_stage_;

  bool has_height_;
  double current_height_cm_;
  double latest_raw_height_cm_;
  double height_min_cm_;
  double height_max_cm_;
  double height_filter_jump_threshold_cm_;
  int height_filter_required_frames_;
  bool has_height_filter_candidate_;
  double height_filter_candidate_cm_;
  int height_filter_candidate_frames_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;
  double emergency_retract_height_threshold_cm_;
  double emergency_retract_z_velocity_threshold_cm_s_;
  bool emergency_servo_retract_active_;

  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;
  std::string vision_mode_topic_;
  std::string route_stage_command_topic_;

  // 视觉对准参数
  double visual_align_pixel_threshold_;
  int visual_align_required_frames_;
  double visual_takeover_timeout_sec_;
  double fine_data_stale_timeout_sec_;

  // 抓取参数
  double pickup_align_altitude_cm_;        // 抓取对准高度（默认 50cm）
  double pickup_grab_altitude_cm_;         // 抓取下探高度（默认 7cm）
  double pickup_hold_at_grab_sec_;         // 在 7cm 悬停吸附的时间（默认 1.0s）
  double pickup_check_altitude_cm_;        // 抓取后判断高度（默认 60cm）
  double pickup_check_observe_sec_;        // 判断高度观察窗口（默认 2.0s）
  double pickup_observe_sec_;              // 兼容旧参数；当前判断窗口由 pickup_check_observe_sec 控制
  int pickup_max_attempts_;                // 抓取最大重试次数（默认 3）
  double circle_lost_window_sec_;          // 兼容旧参数
  double post_pickup_climb_altitude_cm_;

  // 投放参数（独立时序，不受抓取参数影响）
  double drop_altitude_cm_;                // 投放释放高度
  double drop_align_altitude_cm_;          // 投放 AprilTag 对准高度
  double drop_servo_up_settle_sec_;       // 投放到点后舵机收起等待时间
  double drop_servo_down_settle_sec_;     // 投放对准后舵机下放等待时间
  double drop_servo_retract_delay_sec_;   // 断磁铁后收起舵机延迟

  // 视觉接管 / fine_data 状态
  bool visual_takeover_active_;
  bool xy_velocity_hold_active_;
  bool has_fine_data_;
  bool pickup_observed_fine_data_;
  int fine_error_x_px_;
  int fine_error_y_px_;
  rclcpp::Time last_fine_data_time_;

  bool mission_complete_sent_;
  bool failure_hold_active_;

  int aligned_frame_count_;
  rclcpp::Time visual_takeover_start_time_;
  rclcpp::Time last_target_republish_time_;
  double search_approach_altitude_cm_;

  // 抓取/投放子状态机
  TaskPhase phase_;
  rclcpp::Time phase_start_time_;
  int pickup_attempts_;
  bool magnet_sent_in_phase_;              // 当前 *Acting 阶段是否已经发过电磁铁帧

  // 抓取对准成功时记录的实际位置（用于下降/悬停/上升锁位 + 重试回位）
  // 目标 AprilTag 可能不完全在航点 xy，对准后用真实位置代替航点坐标
  double aligned_x_cm_;
  double aligned_y_cm_;
  bool has_aligned_position_;
};

class RouteTestNode : public rclcpp::Node
{
public:
  explicit RouteTestNode(
    const std::shared_ptr<RouteTargetPublisherNode> & route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void detectedPillarsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void pillarDetectionTimeoutCallback();
  void publishPillarDetectionValid(bool valid);
  std::vector<Target> buildRoute(double transit_y_cm) const;
  void loadRoute(const std::vector<Target> & route);

  std::shared_ptr<RouteTargetPublisherNode> route_node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detected_pillars_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pillar_detection_valid_pub_;
  rclcpp::TimerBase::SharedPtr pillar_detection_timeout_timer_;
  std::mutex route_load_mutex_;
  bool use_pillar_detection_ = true;
  std::string device_id_ = "drone1";
  std::string destination_key_ = "delivery_point_1";
  double default_transit_y_cm_ = 186.0;
  double pillar_detection_timeout_sec_ = 20.0;
  bool route_loaded_ = false;
  std::vector<double> mission_x_cm_;
  std::vector<double> mission_y_cm_;
  std::vector<double> mission_z_cm_;
  std::vector<double> mission_yaw_deg_;
  std::vector<bool> mission_use_transit_y_;
  std::vector<int64_t> mission_type_;
  std::vector<int64_t> mission_stage_;
};

}  // namespace activity_control_pkg
