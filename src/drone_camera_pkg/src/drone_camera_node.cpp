#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace
{
std::string defaultTuningFile()
{
  const char * home = std::getenv("HOME");
  if (home == nullptr || home[0] == '\0') {
    return "";
  }
  return std::string(home) + "/.config/nezha/apriltag_detector.yaml";
}
}  // namespace

class DroneCameraNode : public rclcpp::Node
{
public:
  DroneCameraNode()
  : Node("drone_camera_node"),
    camera_device_(declare_parameter<std::string>("camera_device", "/dev/video0")),
    frame_width_(declare_parameter<int>("frame_width", 640)),
    frame_height_(declare_parameter<int>("frame_height", 480)),
    fps_(declare_parameter<double>("fps", 15.0)),
    show_preview_(declare_parameter<bool>("show_preview", false)),
    window_name_(declare_parameter<std::string>("window_name", "drone_camera_preview")),
    fine_data_topic_(declare_parameter<std::string>("fine_data_topic", "/fine_data")),
    vision_mode_topic_(declare_parameter<std::string>("vision_mode_topic", "/vision_target_mode")),
    apriltag_dictionary_name_(declare_parameter<std::string>(
      "apriltag_dictionary", "DICT_APRILTAG_36h11")),
    apriltag_target_id_(declare_parameter<int>("apriltag_target_id", 1)),
    tuning_file_(declare_parameter<std::string>("tuning_file", defaultTuningFile())),
    clahe_clip_limit_(2.0),
    sharpen_amount_(1.0),
    blur_radius_(0),
    tuning_loaded_(false),
    vision_target_mode_(kVisionModeIdle)
  {
    if (apriltag_dictionary_name_ != "DICT_APRILTAG_36h11") {
      throw std::invalid_argument(
              "Only DICT_APRILTAG_36h11 is supported, got " + apriltag_dictionary_name_);
    }
    if (apriltag_target_id_ < 0 || apriltag_target_id_ > 586) {
      throw std::invalid_argument("apriltag_target_id must be in [0, 586]");
    }
    apriltag_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    apriltag_parameters_ = cv::aruco::DetectorParameters::create();
    applyEnhancedDetectorDefaults(apriltag_parameters_);
    loadTuningFile();

    fine_data_pub_ =
      create_publisher<std_msgs::msg::Int32MultiArray>(fine_data_topic_, rclcpp::QoS(10));
    auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    vision_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      vision_mode_topic_, mode_qos,
      std::bind(&DroneCameraNode::visionModeCallback, this, std::placeholders::_1));

    if (!camera_.open(camera_device_)) {
      throw std::runtime_error("Failed to open camera device " + camera_device_);
    }
    if (frame_width_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    }
    if (frame_height_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    }
    if (fps_ > 0.0) {
      camera_.set(cv::CAP_PROP_FPS, fps_);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(fps_, 1.0));
    frame_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DroneCameraNode::frameTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "AprilTag camera started: device=%s preview=%s dictionary=%s pickup_target_id=%d "
      "mode_topic=%s fine_data_topic=%s (mode 1=strict target, mode 2=nearest any id)",
      camera_device_.c_str(), show_preview_ ? "true" : "false",
      apriltag_dictionary_name_.c_str(), apriltag_target_id_,
      vision_mode_topic_.c_str(), fine_data_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "AprilTag tuning: source=%s adaptive=%d..%d/%d corner_refine=%d "
      "CLAHE=%.1f sharpen=%.1f blur_radius=%d",
      tuning_loaded_ ? tuning_file_.c_str() : "enhanced built-in defaults",
      apriltag_parameters_->adaptiveThreshWinSizeMin,
      apriltag_parameters_->adaptiveThreshWinSizeMax,
      apriltag_parameters_->adaptiveThreshWinSizeStep,
      apriltag_parameters_->cornerRefinementMethod,
      clahe_clip_limit_, sharpen_amount_, blur_radius_);
  }

  ~DroneCameraNode() override
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (camera_.isOpened()) {
      camera_.release();
    }
    if (show_preview_) {
      cv::destroyAllWindows();
    }
  }

private:
  static void applyEnhancedDetectorDefaults(
    const cv::Ptr<cv::aruco::DetectorParameters> & parameters)
  {
    parameters->adaptiveThreshWinSizeMin = 3;
    parameters->adaptiveThreshWinSizeMax = 53;
    parameters->adaptiveThreshWinSizeStep = 4;
    parameters->adaptiveThreshConstant = 7.0;
    parameters->minMarkerPerimeterRate = 0.020;
    parameters->maxMarkerPerimeterRate = 4.0;
    parameters->polygonalApproxAccuracyRate = 0.030;
    parameters->minCornerDistanceRate = 0.030;
    parameters->minDistanceToBorder = 3;
    parameters->cornerRefinementMethod = 3;
    parameters->cornerRefinementWinSize = 5;
    parameters->errorCorrectionRate = 0.60;
    parameters->detectInvertedMarker = false;
    parameters->perspectiveRemovePixelPerCell = 6;
    parameters->perspectiveRemoveIgnoredMarginPerCell = 0.13;
  }

  void loadTuningFile()
  {
    if (tuning_file_.empty()) {
      return;
    }

    cv::FileStorage input(tuning_file_, cv::FileStorage::READ);
    if (!input.isOpened()) {
      RCLCPP_WARN(
        get_logger(),
        "AprilTag tuning file not found: %s. Using enhanced defaults; run apriltag_tuner and press s.",
        tuning_file_.c_str());
      return;
    }

    const cv::FileNode root = input["apriltag_tuner"];
    if (root.empty() || !root.isMap()) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid AprilTag tuning file %s: missing apriltag_tuner map. Using enhanced defaults.",
        tuning_file_.c_str());
      return;
    }

    auto tuned = cv::aruco::DetectorParameters::create();
    applyEnhancedDetectorDefaults(tuned);
    double tuned_clahe = clahe_clip_limit_;
    double tuned_sharpen = sharpen_amount_;
    int tuned_blur_radius = blur_radius_;
    int tuned_detect_inverted = 0;

    const auto read_int = [&root](const char * key, int & value) {
        const cv::FileNode node = root[key];
        if (!node.empty()) {
          node >> value;
        }
      };
    const auto read_double = [&root](const char * key, double & value) {
        const cv::FileNode node = root[key];
        if (!node.empty()) {
          node >> value;
        }
      };

    read_int("adaptive_thresh_win_size_min", tuned->adaptiveThreshWinSizeMin);
    read_int("adaptive_thresh_win_size_max", tuned->adaptiveThreshWinSizeMax);
    read_int("adaptive_thresh_win_size_step", tuned->adaptiveThreshWinSizeStep);
    read_double("adaptive_thresh_constant", tuned->adaptiveThreshConstant);
    read_double("min_marker_perimeter_rate", tuned->minMarkerPerimeterRate);
    read_double("max_marker_perimeter_rate", tuned->maxMarkerPerimeterRate);
    read_double("polygonal_approx_accuracy_rate", tuned->polygonalApproxAccuracyRate);
    read_double("min_corner_distance_rate", tuned->minCornerDistanceRate);
    read_int("min_distance_to_border", tuned->minDistanceToBorder);
    read_int("corner_refinement_method", tuned->cornerRefinementMethod);
    read_int("corner_refinement_win_size", tuned->cornerRefinementWinSize);
    read_double("error_correction_rate", tuned->errorCorrectionRate);
    read_int("detect_inverted_marker", tuned_detect_inverted);
    read_int("perspective_remove_pixel_per_cell", tuned->perspectiveRemovePixelPerCell);
    read_double(
      "perspective_remove_ignored_margin_per_cell",
      tuned->perspectiveRemoveIgnoredMarginPerCell);
    read_double("clahe_clip_limit", tuned_clahe);
    read_double("sharpen_amount", tuned_sharpen);
    read_int("blur_radius", tuned_blur_radius);

    tuned->adaptiveThreshWinSizeMin = std::max(tuned->adaptiveThreshWinSizeMin, 3);
    if (tuned->adaptiveThreshWinSizeMin % 2 == 0) {
      ++tuned->adaptiveThreshWinSizeMin;
    }
    tuned->adaptiveThreshWinSizeMax = std::max(
      tuned->adaptiveThreshWinSizeMax, tuned->adaptiveThreshWinSizeMin);
    if (tuned->adaptiveThreshWinSizeMax % 2 == 0) {
      ++tuned->adaptiveThreshWinSizeMax;
    }
    tuned->adaptiveThreshWinSizeStep = std::max(tuned->adaptiveThreshWinSizeStep, 1);
    tuned->minMarkerPerimeterRate = std::max(tuned->minMarkerPerimeterRate, 0.001);
    tuned->maxMarkerPerimeterRate = std::max(
      tuned->maxMarkerPerimeterRate, tuned->minMarkerPerimeterRate + 0.01);
    tuned->polygonalApproxAccuracyRate = std::max(
      tuned->polygonalApproxAccuracyRate, 0.001);
    tuned->minCornerDistanceRate = std::max(tuned->minCornerDistanceRate, 0.001);
    tuned->minDistanceToBorder = std::max(tuned->minDistanceToBorder, 0);
    tuned->cornerRefinementMethod = std::clamp(tuned->cornerRefinementMethod, 0, 3);
    tuned->cornerRefinementWinSize = std::max(tuned->cornerRefinementWinSize, 1);
    tuned->errorCorrectionRate = std::clamp(tuned->errorCorrectionRate, 0.0, 1.0);
    tuned->detectInvertedMarker = tuned_detect_inverted != 0;
    tuned->perspectiveRemovePixelPerCell = std::max(tuned->perspectiveRemovePixelPerCell, 1);
    tuned->perspectiveRemoveIgnoredMarginPerCell = std::clamp(
      tuned->perspectiveRemoveIgnoredMarginPerCell, 0.0, 0.5);

    apriltag_parameters_ = tuned;
    clahe_clip_limit_ = std::max(tuned_clahe, 0.0);
    sharpen_amount_ = std::clamp(tuned_sharpen, 0.0, 4.0);
    blur_radius_ = std::clamp(tuned_blur_radius, 0, 5);
    tuning_loaded_ = true;
  }

  cv::Mat preprocessForDetection(const cv::Mat & frame) const
  {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    if (clahe_clip_limit_ > 0.0) {
      cv::Mat equalized;
      auto clahe = cv::createCLAHE(clahe_clip_limit_, cv::Size(8, 8));
      clahe->apply(gray, equalized);
      gray = equalized;
    }
    if (blur_radius_ > 0) {
      const int kernel = blur_radius_ * 2 + 1;
      cv::GaussianBlur(gray, gray, cv::Size(kernel, kernel), 0.0);
    }
    if (sharpen_amount_ > 0.0) {
      cv::Mat blurred;
      cv::Mat sharpened;
      cv::GaussianBlur(gray, blurred, cv::Size(0, 0), 1.0);
      cv::addWeighted(
        gray, 1.0 + sharpen_amount_, blurred, -sharpen_amount_, 0.0, sharpened);
      gray = sharpened;
    }
    return gray;
  }

  void visionModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (msg->data > kVisionModeDropAnyAprilTag) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring unsupported vision mode: %u",
        static_cast<unsigned int>(msg->data));
      return;
    }
    if (vision_target_mode_ != msg->data) {
      vision_target_mode_ = msg->data;
      RCLCPP_INFO(
        get_logger(), "Vision mode changed to %u",
        static_cast<unsigned int>(vision_target_mode_));
    }
  }

  void publishFineDataFromCenter(const cv::Mat & frame, const cv::Point2f & center)
  {
    const float image_center_x = static_cast<float>(frame.cols) / 2.0F;
    const float image_center_y = static_cast<float>(frame.rows) / 2.0F;
    std_msgs::msg::Int32MultiArray msg;
    msg.data = {
      static_cast<int>(std::lround(image_center_y - center.y)),
      static_cast<int>(std::lround(image_center_x - center.x))};
    fine_data_pub_->publish(msg);
  }

  void detectAprilTagAndPublish(cv::Mat & frame, bool strict_pickup_id)
  {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    const cv::Mat detection_frame = preprocessForDetection(frame);
    cv::aruco::detectMarkers(
      detection_frame, apriltag_dictionary_, corners, ids, apriltag_parameters_);
    if (ids.empty()) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "No AprilTag found.");
      return;
    }

    const cv::Point2f image_center(
      static_cast<float>(frame.cols) / 2.0F,
      static_cast<float>(frame.rows) / 2.0F);
    int best_index = -1;
    double best_distance = std::numeric_limits<double>::max();
    cv::Point2f best_center;
    for (std::size_t index = 0; index < ids.size(); ++index) {
      if (strict_pickup_id && ids[index] != apriltag_target_id_) {
        continue;
      }
      if (corners[index].size() != 4) {
        continue;
      }
      cv::Point2f center(0.0F, 0.0F);
      for (const auto & corner : corners[index]) {
        center += corner;
      }
      center *= 0.25F;
      const double distance = std::hypot(
        static_cast<double>(center.x - image_center.x),
        static_cast<double>(center.y - image_center.y));
      if (distance < best_distance) {
        best_distance = distance;
        best_index = static_cast<int>(index);
        best_center = center;
      }
    }

    if (best_index < 0) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "AprilTags detected, but pickup target id %d is absent.", apriltag_target_id_);
      return;
    }

    publishFineDataFromCenter(frame, best_center);
    cv::aruco::drawDetectedMarkers(frame, corners, ids);
    cv::circle(
      frame,
      cv::Point(
        static_cast<int>(std::lround(best_center.x)),
        static_cast<int>(std::lround(best_center.y))),
      4, cv::Scalar(0, 0, 255), cv::FILLED);
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000, "Selected AprilTag id=%d distance=%.1fpx mode=%u",
      ids[static_cast<std::size_t>(best_index)], best_distance,
      static_cast<unsigned int>(vision_target_mode_));
  }

  void frameTimerCallback()
  {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!camera_.isOpened() || !camera_.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to read camera frame.");
        return;
      }
    }
    if (vision_target_mode_ == kVisionModePickupAprilTag) {
      detectAprilTagAndPublish(frame, true);
    } else if (vision_target_mode_ == kVisionModeDropAnyAprilTag) {
      detectAprilTagAndPublish(frame, false);
    }
    if (show_preview_) {
      cv::imshow(window_name_, frame);
      cv::waitKey(1);
    }
  }

  std::string camera_device_;
  int frame_width_;
  int frame_height_;
  double fps_;
  bool show_preview_;
  std::string window_name_;
  std::string fine_data_topic_;
  std::string vision_mode_topic_;
  std::string apriltag_dictionary_name_;
  int apriltag_target_id_;
  std::string tuning_file_;
  double clahe_clip_limit_;
  double sharpen_amount_;
  int blur_radius_;
  bool tuning_loaded_;
  uint8_t vision_target_mode_;
  std::mutex frame_mutex_;
  cv::VideoCapture camera_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vision_mode_sub_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
  cv::Ptr<cv::aruco::Dictionary> apriltag_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> apriltag_parameters_;

  static constexpr uint8_t kVisionModeIdle = 0;
  static constexpr uint8_t kVisionModePickupAprilTag = 1;
  static constexpr uint8_t kVisionModeDropAnyAprilTag = 2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
