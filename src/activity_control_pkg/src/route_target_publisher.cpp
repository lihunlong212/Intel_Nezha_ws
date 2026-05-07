#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <chrono>
#include <clocale>
#include <cmath>
#include <functional>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;

const char * phaseToString(TaskPhase phase)
{
  switch (phase) {
    case TaskPhase::Idle: return "Idle";
    case TaskPhase::PickupAligning: return "PickupAligning";
    case TaskPhase::PickupDescending: return "PickupDescending";
    case TaskPhase::PickupHolding: return "PickupHolding";
    case TaskPhase::PickupAscending: return "PickupAscending";
    case TaskPhase::PickupObserving: return "PickupObserving";
    case TaskPhase::DropArriving: return "DropArriving";
    case TaskPhase::DropActing: return "DropActing";
  }
  return "?";
}
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0),
  visual_align_pixel_threshold_(0.0),
  visual_align_required_frames_(0),
  visual_takeover_timeout_sec_(0.0),
  fine_data_stale_timeout_sec_(0.0),
  pickup_align_altitude_cm_(0.0),
  pickup_grab_altitude_cm_(0.0),
  pickup_hold_at_grab_sec_(0.0),
  pickup_observe_sec_(0.0),
  pickup_max_attempts_(0),
  circle_lost_window_sec_(0.0),
  drop_altitude_cm_(0.0),
  drop_servo_down_duration_sec_(0.0),
  drop_magnet_off_delay_sec_(0.0),
  visual_takeover_active_(false),
  has_fine_data_(false),
  fine_error_x_px_(0),
  fine_error_y_px_(0),
  mission_complete_sent_(false),
  aligned_frame_count_(0),
  phase_(TaskPhase::Idle),
  pickup_attempts_(0),
  magnet_sent_in_phase_(false)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");

  visual_align_pixel_threshold_ = declare_parameter("visual_align_pixel_threshold", 100.0);
  visual_align_required_frames_ = declare_parameter("visual_align_required_frames", 3);
  visual_takeover_timeout_sec_ = declare_parameter("visual_takeover_timeout_sec", 15.0);
  fine_data_stale_timeout_sec_ = declare_parameter("fine_data_stale_timeout_sec", 0.5);

  // 抓取参数
  pickup_align_altitude_cm_ = declare_parameter("pickup_align_altitude_cm", 50.0);
  pickup_grab_altitude_cm_ = declare_parameter("pickup_grab_altitude_cm", 20.0);
  pickup_hold_at_grab_sec_ = declare_parameter("pickup_hold_at_grab_sec", 1.0);
  pickup_observe_sec_ = declare_parameter("pickup_observe_sec", 1.0);
  pickup_max_attempts_ = declare_parameter("pickup_max_attempts", 3);
  circle_lost_window_sec_ = declare_parameter("circle_lost_window_sec", 1.0);
  // 投放参数（独立）
  drop_altitude_cm_ = declare_parameter("drop_altitude_cm", 50.0);
  drop_servo_down_duration_sec_ = declare_parameter("drop_servo_down_duration_sec", 2.0);
  drop_magnet_off_delay_sec_ = declare_parameter("drop_magnet_off_delay_sec", 1.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  visual_takeover_active_pub_ =
    create_publisher<std_msgs::msg::Bool>("/visual_takeover_active", durable_qos);
  servo_control_pub_ =
    create_publisher<std_msgs::msg::UInt8>("/servo_control", rclcpp::QoS(10).reliable());
  electromagnet_control_pub_ =
    create_publisher<std_msgs::msg::UInt8>("/electromagnet_control", rclcpp::QoS(10).reliable());
  mission_complete_pub_ =
    create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));
  fine_data_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/fine_data",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::fineDataCallback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  publishVisualTakeoverState(false);

  RCLCPP_INFO(
    get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s",
    map_frame_.c_str(), laser_link_frame_.c_str(), output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_, yaw_tol_deg_, height_tol_cm_);
  RCLCPP_INFO(
    get_logger(),
    "Visual align: threshold=%.1fpx frames=%d timeout=%.1fs stale=%.1fs",
    visual_align_pixel_threshold_, visual_align_required_frames_,
    visual_takeover_timeout_sec_, fine_data_stale_timeout_sec_);
  RCLCPP_INFO(
    get_logger(),
    "Pickup: align_z=%.1fcm grab_z=%.1fcm hold=%.1fs observe=%.1fs max_attempts=%d",
    pickup_align_altitude_cm_, pickup_grab_altitude_cm_,
    pickup_hold_at_grab_sec_, pickup_observe_sec_, pickup_max_attempts_);
  RCLCPP_INFO(
    get_logger(),
    "Drop: z=%.1fcm servo_down=%.1fs magnet_off_delay=%.1fs",
    drop_altitude_cm_, drop_servo_down_duration_sec_, drop_magnet_off_delay_sec_);
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  const bool was_completed =
    current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size();
  targets_.push_back(target);
  if (was_empty || was_completed) {
    mission_complete_sent_ = false;
    current_idx_ = was_completed ? targets_.size() - 1 : 0;
    publishCurrent();
  }
}

std::size_t RouteTargetPublisherNode::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

std::size_t RouteTargetPublisherNode::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

void RouteTargetPublisherNode::publishCurrent()
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    publishTarget(getPublishedTarget(targets_[current_idx_]), current_idx_ == 0);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  target_pub_->publish(message);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;
  active_controller_pub_->publish(active_msg);

  RCLCPP_INFO(
    get_logger(),
    "Published target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg type=%d%s",
    target.x_cm, target.y_cm, target.z_cm, target.yaw_deg, target.type,
    init_flag ? " (first)" : "");
}

Target RouteTargetPublisherNode::getPublishedTarget(const Target & target) const
{
  Target published_target = target;
  switch (phase_) {
    case TaskPhase::PickupAligning:
    case TaskPhase::PickupAscending:
    case TaskPhase::PickupObserving:
      published_target.z_cm = pickup_align_altitude_cm_;
      break;
    case TaskPhase::PickupDescending:
    case TaskPhase::PickupHolding:
      published_target.z_cm = pickup_grab_altitude_cm_;
      break;
    case TaskPhase::DropArriving:
    case TaskPhase::DropActing:
      published_target.z_cm = drop_altitude_cm_;
      break;
    case TaskPhase::Idle:
    default:
      // 普通巡航高度由 target 自身 z_cm 决定
      break;
  }
  return published_target;
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Route monitor received /height: %.1fcm",
    current_height_cm_);
}

void RouteTargetPublisherNode::fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/fine_data requires 2 values [x_px, y_px]");
    return;
  }

  fine_error_x_px_ = msg->data[0];
  fine_error_y_px_ = msg->data[1];
  has_fine_data_ = true;
  last_fine_data_time_ = now();
}

bool RouteTargetPublisherNode::getCurrentPose(
  double & x_cm,
  double & y_cm,
  double & z_cm,
  double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  const bool z_ok = std::fabs(dz) <= height_tol_cm_;
  const bool xy_ok = dxy <= pos_tol_cm_;
  const bool yaw_ok = std::fabs(dyaw) <= yaw_tol_deg_;

  if (target.z_cm > 20.0) {
    if (current_idx_ == 0) {
      return z_ok;
    }
    return z_ok && xy_ok;
  }

  return z_ok && xy_ok && yaw_ok;
}

bool RouteTargetPublisherNode::hasFreshFineData(const rclcpp::Time & now_time) const
{
  if (!has_fine_data_ || last_fine_data_time_.nanoseconds() == 0) {
    return false;
  }
  return (now_time - last_fine_data_time_).seconds() <= fine_data_stale_timeout_sec_;
}

void RouteTargetPublisherNode::advanceToNextTarget()
{
  const std::size_t previous_idx = current_idx_;
  ++current_idx_;
  if (current_idx_ < targets_.size()) {
    RCLCPP_INFO(
      get_logger(),
      "Advance target: %zu -> %zu",
      previous_idx,
      current_idx_);
    publishCurrent();
  } else {
    current_idx_ = targets_.size();
    if (!mission_complete_sent_ && mission_complete_pub_) {
      std_msgs::msg::Empty mission_complete_msg;
      mission_complete_pub_->publish(mission_complete_msg);
      mission_complete_sent_ = true;
    }
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO(get_logger(), "All targets completed.");
  }
}

void RouteTargetPublisherNode::publishVisualTakeoverState(bool active)
{
  std_msgs::msg::Bool msg;
  msg.data = active;
  visual_takeover_active_pub_->publish(msg);
}

void RouteTargetPublisherNode::publishServoControl(uint8_t state)
{
  std_msgs::msg::UInt8 msg;
  msg.data = state;
  servo_control_pub_->publish(msg);
}

void RouteTargetPublisherNode::publishElectromagnetControl(uint8_t state)
{
  std_msgs::msg::UInt8 msg;
  msg.data = state;
  electromagnet_control_pub_->publish(msg);
}

void RouteTargetPublisherNode::setPhase(TaskPhase phase, const rclcpp::Time & now_time)
{
  if (phase_ != phase) {
    RCLCPP_INFO(get_logger(), "Phase: %s -> %s (target %zu)",
      phaseToString(phase_), phaseToString(phase), current_idx_);
  }
  phase_ = phase;
  phase_start_time_ = now_time;
  magnet_sent_in_phase_ = false;

  // 视觉接管：抓取流程的全部 5 个阶段都开启。
  // 舵机下放期间 /fine_data 过期 → PID 让 XY 速度归零，无人机原地保持视觉对准过的位置。
  // 上升回 50cm 后摄像头解除遮挡，若仍有黑圆，PID 会把无人机拉回黑圆上方。
  // 投放阶段不开启视觉接管，使用位置 PID 直接飞到 (x,y,50)。
  const bool takeover =
    phase == TaskPhase::PickupAligning ||
    phase == TaskPhase::PickupDescending ||
    phase == TaskPhase::PickupHolding ||
    phase == TaskPhase::PickupAscending ||
    phase == TaskPhase::PickupObserving;
  if (visual_takeover_active_ != takeover) {
    visual_takeover_active_ = takeover;
    publishVisualTakeoverState(takeover);
  }

  // 进入 PickupAligning 时重置帧计数 + 视觉超时起点（重试场景必须重置）
  if (phase == TaskPhase::PickupAligning) {
    aligned_frame_count_ = 0;
    visual_takeover_start_time_ = now_time;
  }

  // 下发新阶段对应的目标位置（z 由 getPublishedTarget 调整）
  if (current_idx_ < targets_.size()) {
    publishTarget(getPublishedTarget(targets_[current_idx_]), false);
  }
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size()) {
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    if (visual_takeover_active_) {
      visual_takeover_active_ = false;
      publishVisualTakeoverState(false);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "All targets completed. Keeping stop signal active.");
    return;
  }

  if (current_idx_ == std::numeric_limits<std::size_t>::max()) {
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target & target = targets_[current_idx_];
  const rclcpp::Time now_time = now();
  const double phase_elapsed = (now_time - phase_start_time_).seconds();

  switch (phase_) {
    case TaskPhase::Idle: {
      const double dx_now = target.x_cm - x_cm;
      const double dy_now = target.y_cm - y_cm;
      const double dxy_now = std::hypot(dx_now, dy_now);
      const double dz_now = target.z_cm - z_cm;
      const bool reached = isReached(target, x_cm, y_cm, z_cm, yaw_deg);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Idle target %zu type=%d: tgt=(%.1f,%.1f,%.1f) cur=(%.1f,%.1f,%.1f) dxy=%.1f dz=%.1f tol_xy=%.1f tol_z=%.1f has_height=%s reached=%s",
        current_idx_, target.type,
        target.x_cm, target.y_cm, target.z_cm,
        x_cm, y_cm, z_cm,
        dxy_now, dz_now, pos_tol_cm_, height_tol_cm_,
        has_height_ ? "true" : "false",
        reached ? "YES" : "no");

      if (!reached) {
        return;
      }

      const double dx = target.x_cm - x_cm;
      const double dy = target.y_cm - y_cm;
      const double dz = target.z_cm - z_cm;
      const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);
      RCLCPP_INFO(
        get_logger(),
        "Target %zu reached: pos_err=(%.1f, %.1f, %.1f)cm yaw_err=%.1fdeg type=%d",
        current_idx_, dx, dy, dz, dyaw, target.type);

      if (target.type == 2) {
        pickup_attempts_ = 0;
        setPhase(TaskPhase::PickupAligning, now_time);
      } else if (target.type == 3) {
        setPhase(TaskPhase::DropArriving, now_time);
      } else {
        advanceToNextTarget();
      }
      return;
    }

    case TaskPhase::PickupAligning: {
      if (phase_elapsed > visual_takeover_timeout_sec_) {
        RCLCPP_WARN(
          get_logger(),
          "Pickup alignment timed out for target %zu after %.1fs (attempt %d/%d). Giving up.",
          current_idx_, phase_elapsed, pickup_attempts_ + 1, pickup_max_attempts_);
        publishElectromagnetControl(0x00);
        publishServoControl(0x00);
        setPhase(TaskPhase::Idle, now_time);
        advanceToNextTarget();
        return;
      }

      if (!hasFreshFineData(now_time)) {
        aligned_frame_count_ = 0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Waiting for fresh /fine_data while aligning at target %zu.", current_idx_);
        return;
      }

      const double pixel_radius = std::hypot(
        static_cast<double>(fine_error_x_px_),
        static_cast<double>(fine_error_y_px_));
      const double height_error_cm = pickup_align_altitude_cm_ - z_cm;
      const bool height_ok = std::fabs(height_error_cm) <= height_tol_cm_;
      const bool xy_ok = pixel_radius < visual_align_pixel_threshold_;

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PickupAligning %zu: x_px=%d y_px=%d r=%.1f thr=%.1f h_err=%.1fcm z=%.1fcm frames=%d/%d attempt=%d/%d",
        current_idx_, fine_error_x_px_, fine_error_y_px_,
        pixel_radius, visual_align_pixel_threshold_,
        height_error_cm, z_cm,
        aligned_frame_count_, visual_align_required_frames_,
        pickup_attempts_ + 1, pickup_max_attempts_);

      if (xy_ok && height_ok) {
        ++aligned_frame_count_;
        if (aligned_frame_count_ >= visual_align_required_frames_) {
          // 对准成功：电磁铁通电（仅此一次，整个抓取期间保持通电），舵机下放，开始下降到 20cm
          publishElectromagnetControl(0x01);
          publishServoControl(0x01);
          setPhase(TaskPhase::PickupDescending, now_time);
        }
      } else {
        aligned_frame_count_ = 0;
      }
      return;
    }

    case TaskPhase::PickupDescending: {
      const double height_error_cm = pickup_grab_altitude_cm_ - z_cm;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "PickupDescending %zu: z=%.1fcm target=%.1fcm err=%.1fcm",
        current_idx_, z_cm, pickup_grab_altitude_cm_, height_error_cm);
      if (std::fabs(height_error_cm) <= height_tol_cm_) {
        setPhase(TaskPhase::PickupHolding, now_time);
      }
      return;
    }

    case TaskPhase::PickupHolding: {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "PickupHolding %zu: elapsed=%.2fs / %.2fs (magnet ON, grabbing)",
        current_idx_, phase_elapsed, pickup_hold_at_grab_sec_);
      if (phase_elapsed >= pickup_hold_at_grab_sec_) {
        // 抓住后舵机收起开始上升；电磁铁保持通电不变
        publishServoControl(0x00);
        setPhase(TaskPhase::PickupAscending, now_time);
      }
      return;
    }

    case TaskPhase::PickupAscending: {
      const double height_error_cm = pickup_align_altitude_cm_ - z_cm;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "PickupAscending %zu: z=%.1fcm target=%.1fcm err=%.1fcm (magnet still ON)",
        current_idx_, z_cm, pickup_align_altitude_cm_, height_error_cm);
      if (std::fabs(height_error_cm) <= height_tol_cm_) {
        setPhase(TaskPhase::PickupObserving, now_time);
      }
      return;
    }

    case TaskPhase::PickupObserving: {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "PickupObserving %zu: elapsed=%.2fs / %.2fs",
        current_idx_, phase_elapsed, pickup_observe_sec_);
      if (phase_elapsed < pickup_observe_sec_) {
        return;
      }

      // 成败判定：观察期内是否还能在 /fine_data 上看到黑圆
      // 抓取成功 → 黑圆被吸走 → /fine_data 过期 → circle_seen=false
      // 抓取失败 → 黑圆还在地上 → /fine_data 持续刷新 → circle_seen=true
      const bool circle_seen =
        hasFreshFineData(now_time) &&
        (now_time - last_fine_data_time_).seconds() <= circle_lost_window_sec_;

      if (!circle_seen) {
        RCLCPP_INFO(
          get_logger(),
          "Pickup SUCCESS at target %zu (attempt %d/%d). Advancing.",
          current_idx_, pickup_attempts_ + 1, pickup_max_attempts_);
        setPhase(TaskPhase::Idle, now_time);
        advanceToNextTarget();
        return;
      }

      ++pickup_attempts_;
      if (pickup_attempts_ >= pickup_max_attempts_) {
        RCLCPP_WARN(
          get_logger(),
          "Pickup FAILED after %d attempts at target %zu. Giving up.",
          pickup_attempts_, current_idx_);
        publishElectromagnetControl(0x00);
        publishServoControl(0x00);
        setPhase(TaskPhase::Idle, now_time);
        advanceToNextTarget();
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Pickup retry at target %zu (attempt %d/%d). Magnet stays ON.",
          current_idx_, pickup_attempts_ + 1, pickup_max_attempts_);
        // 重试：电磁铁保持通电（用户要求一次 0x01 持续到任务结束），重新对准抓取
        setPhase(TaskPhase::PickupAligning, now_time);
      }
      return;
    }

    case TaskPhase::DropArriving: {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DropArriving %zu: target=(%.1f,%.1f,%.1f) current=(%.1f,%.1f,%.1f)",
        current_idx_, target.x_cm, target.y_cm, drop_altitude_cm_, x_cm, y_cm, z_cm);

      Target drop_target = target;
      drop_target.z_cm = drop_altitude_cm_;
      if (isReached(drop_target, x_cm, y_cm, z_cm, yaw_deg)) {
        publishServoControl(0x01);
        setPhase(TaskPhase::DropActing, now_time);
      }
      return;
    }

    case TaskPhase::DropActing: {
      // 时序：t=0 已发 servo=01；t=drop_magnet_off_delay 发 magnet=00；t=drop_servo_down_duration 发 servo=00 → 离开
      if (!magnet_sent_in_phase_ && phase_elapsed >= drop_magnet_off_delay_sec_) {
        publishElectromagnetControl(0x00);
        magnet_sent_in_phase_ = true;
        RCLCPP_INFO(get_logger(), "DropActing: magnet OFF at t=%.2fs", phase_elapsed);
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "DropActing %zu: elapsed=%.2fs / %.2fs (magnet_sent=%s)",
        current_idx_, phase_elapsed, drop_servo_down_duration_sec_,
        magnet_sent_in_phase_ ? "true" : "false");

      if (phase_elapsed >= drop_servo_down_duration_sec_) {
        if (!magnet_sent_in_phase_) {
          publishElectromagnetControl(0x00);
          magnet_sent_in_phase_ = true;
          RCLCPP_WARN(get_logger(),
            "DropActing: magnet_delay >= servo_down_duration, forcing magnet OFF now");
        }
        publishServoControl(0x00);
        RCLCPP_INFO(get_logger(), "Drop completed at target %zu. Advancing.", current_idx_);
        setPhase(TaskPhase::Idle, now_time);
        advanceToNextTarget();
      }
      return;
    }
  }
}

double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}

RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node)
{
  std::setlocale(LC_ALL, "");

  const auto route = buildRoute();
  if (route.empty()) {
    RCLCPP_ERROR(get_logger(), "Default route is empty. Nothing to load.");
    return;
  }

  loadRoute(route);
}

std::vector<Target> RouteTestNode::buildRoute() const
{
  // 航点 type: 1=普通  2=抓取  3=投放
  return std::vector<Target>{
    Target{0.0,   0.0,   130.0, 0.0, 1},   // 起飞 / 巡航高度
    Target{125.0, 100.0, 130.0, 0.0, 2},   // 抓取
    Target{0.0,   0.0,   130.0, 0.0, 3},   // 投放（坐标按需调整）
    Target{0.0,   0.0,   0.0,   0.0, 1},   // 落地
  };
}

void RouteTestNode::loadRoute(const std::vector<Target> & route)
{
  RCLCPP_INFO(
    get_logger(),
    "Loading default route with %zu targets.",
    route.size());

  for (std::size_t index = 0; index < route.size(); ++index) {
    const auto & target = route[index];
    route_node_->addTarget(target);
    RCLCPP_INFO(
      get_logger(),
      "Loaded target %zu/%zu: x=%.1f y=%.1f z=%.1f yaw=%.1f type=%d",
      index + 1, route.size(),
      target.x_cm, target.y_cm, target.z_cm, target.yaw_deg, target.type);
  }

  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(
    get_logger(),
    "Route is now active. Current target index=%zu",
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));
}

}  // namespace activity_control_pkg
