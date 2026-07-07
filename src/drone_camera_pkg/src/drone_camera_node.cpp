#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

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
    blue_hue_min_(declare_parameter<int>("blue_hue_min", 58)),
    blue_hue_max_(declare_parameter<int>("blue_hue_max", 111)),
    blue_saturation_min_(declare_parameter<int>("blue_saturation_min", 49)),
    blue_value_min_(declare_parameter<int>("blue_value_min", 75)),
    min_square_area_(declare_parameter<double>(
      "min_square_area", declare_parameter<double>("min_circle_area", 340.0))),
    min_square_fill_ratio_(declare_parameter<double>("min_square_fill_ratio", 0.22)),
    apriltag_dictionary_name_(declare_parameter<std::string>("apriltag_dictionary", "DICT_APRILTAG_36h11")),
    apriltag_target_id_(declare_parameter<int>("apriltag_target_id", -1)),
    vision_target_mode_(kVisionModeBlackSquare)
  {
    apriltag_dictionary_ = cv::aruco::getPredefinedDictionary(
      dictionaryFromName(apriltag_dictionary_name_));
    apriltag_parameters_ = cv::aruco::DetectorParameters::create();

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
      "Camera node started. camera_device=%s show_preview=%s fine_data_topic=%s vision_mode_topic=%s blue_hsv=[%d,%d] s_min=%d v_min=%d min_square_area=%.1f min_square_fill_ratio=%.2f apriltag_dictionary=%s apriltag_target_id=%d",
      camera_device_.c_str(),
      show_preview_ ? "true" : "false",
      fine_data_topic_.c_str(),
      vision_mode_topic_.c_str(),
      blue_hue_min_,
      blue_hue_max_,
      blue_saturation_min_,
      blue_value_min_,
      min_square_area_,
      min_square_fill_ratio_,
      apriltag_dictionary_name_.c_str(),
      apriltag_target_id_);
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
  struct SquareCandidate
  {
    int contour_index{-1};
    cv::Point2f center{};
    std::vector<cv::Point> approx{};
    double area{0.0};
    double score{0.0};
  };

  static cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromName(const std::string & name)
  {
    if (name == "DICT_APRILTAG_16h5") {
      return cv::aruco::DICT_APRILTAG_16h5;
    }
    if (name == "DICT_APRILTAG_25h9") {
      return cv::aruco::DICT_APRILTAG_25h9;
    }
    if (name == "DICT_APRILTAG_36h10") {
      return cv::aruco::DICT_APRILTAG_36h10;
    }
    return cv::aruco::DICT_APRILTAG_36h11;
  }

  void visionModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    const uint8_t mode = msg->data;
    if (mode > kVisionModeAprilTag) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring unsupported vision target mode: %u", static_cast<unsigned int>(mode));
      return;
    }
    if (vision_target_mode_ == mode) {
      return;
    }
    vision_target_mode_ = mode;
    RCLCPP_INFO(
      get_logger(), "Vision target mode changed to %u",
      static_cast<unsigned int>(vision_target_mode_));
  }

  void publishFineDataFromCenter(const cv::Mat & frame, const cv::Point2f & center)
  {
    const float image_center_x = static_cast<float>(frame.cols) / 2.0F;
    const float image_center_y = static_cast<float>(frame.rows) / 2.0F;
    const int x_offset = static_cast<int>(std::lround(image_center_y - center.y));
    const int y_offset = static_cast<int>(std::lround(image_center_x - center.x));

    std_msgs::msg::Int32MultiArray fine_data_msg;
    fine_data_msg.data = {x_offset, y_offset};
    fine_data_pub_->publish(fine_data_msg);
  }

  static double cornerCosine(
    const cv::Point & previous, const cv::Point & corner, const cv::Point & next)
  {
    const double ux = static_cast<double>(previous.x - corner.x);
    const double uy = static_cast<double>(previous.y - corner.y);
    const double vx = static_cast<double>(next.x - corner.x);
    const double vy = static_cast<double>(next.y - corner.y);
    const double denominator = std::hypot(ux, uy) * std::hypot(vx, vy);
    if (denominator <= std::numeric_limits<double>::epsilon()) {
      return 1.0;
    }
    return std::abs((ux * vx + uy * vy) / denominator);
  }

  void detectBlueSquareAndPublish(cv::Mat & frame)
  {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::GaussianBlur(hsv, hsv, cv::Size(5, 5), 0.0);
    cv::Mat mask;
    const int hue_min = std::max(0, std::min(blue_hue_min_, 179));
    const int hue_max = std::max(0, std::min(blue_hue_max_, 179));
    const int saturation_min = std::max(0, std::min(blue_saturation_min_, 255));
    const int value_min = std::max(0, std::min(blue_value_min_, 255));
    cv::inRange(
      hsv,
      cv::Scalar(hue_min, saturation_min, value_min),
      cv::Scalar(hue_max, 255, 255),
      mask);

    const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    const cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    SquareCandidate best;
    const double min_area = std::max(20.0, min_square_area_);
    const double max_aspect_ratio = std::max(1.0, kMaxSquareAspectRatio);
    const double min_fill_ratio = std::max(0.05, std::min(min_square_fill_ratio_, 1.0));
    for (std::size_t i = 0; i < contours.size(); ++i) {
      const double area = std::abs(cv::contourArea(contours[i]));
      if (area < min_area) {
        continue;
      }

      const cv::Rect bounds = cv::boundingRect(contours[i]);
      if (bounds.width < 8 || bounds.height < 8) {
        continue;
      }

      const double aspect_ratio =
        static_cast<double>(std::max(bounds.width, bounds.height)) /
        static_cast<double>(std::max(1, std::min(bounds.width, bounds.height)));
      if (aspect_ratio > max_aspect_ratio) {
        continue;
      }

      const double perimeter = cv::arcLength(contours[i], true);
      if (perimeter <= 0.0) {
        continue;
      }

      std::vector<cv::Point> approx;
      cv::approxPolyDP(contours[i], approx, 0.02 * perimeter, true);
      if (approx.size() != 4 || !cv::isContourConvex(approx)) {
        continue;
      }

      double max_corner_cosine = 0.0;
      for (std::size_t corner_index = 0; corner_index < approx.size(); ++corner_index) {
        const cv::Point & previous = approx[(corner_index + approx.size() - 1) % approx.size()];
        const cv::Point & corner = approx[corner_index];
        const cv::Point & next = approx[(corner_index + 1) % approx.size()];
        max_corner_cosine = std::max(max_corner_cosine, cornerCosine(previous, corner, next));
      }
      if (max_corner_cosine > kMaxSquareCornerCosine) {
        continue;
      }

      const double rect_area = static_cast<double>(bounds.width) * static_cast<double>(bounds.height);
      const double fill_ratio = area / std::max(1.0, rect_area);
      if (fill_ratio < min_fill_ratio) {
        continue;
      }

      const cv::Moments moments = cv::moments(contours[i]);
      if (std::abs(moments.m00) <= std::numeric_limits<double>::epsilon()) {
        continue;
      }

      const double aspect_score = 1.0 / aspect_ratio;
      const double fill_score = std::min(1.0, fill_ratio);
      const double corner_score = 1.0 - std::min(1.0, max_corner_cosine / kMaxSquareCornerCosine);
      const double shape_score =
        0.40 * fill_score + 0.35 * aspect_score + 0.25 * corner_score;
      const double score = area * shape_score;
      if (score > best.score) {
        best.contour_index = static_cast<int>(i);
        best.center = cv::Point2f(
          static_cast<float>(moments.m10 / moments.m00),
          static_cast<float>(moments.m01 / moments.m00));
        best.approx = approx;
        best.area = area;
        best.score = score;
      }
    }

    if (best.contour_index < 0) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No blue square found. hue=[%d,%d] s_min=%d v_min=%d contours=%zu",
        hue_min, hue_max, saturation_min, value_min, contours.size());
      return;
    }

    publishFineDataFromCenter(frame, best.center);

    cv::drawContours(frame, contours, best.contour_index, cv::Scalar(0, 255, 0), 2);
    std::vector<std::vector<cv::Point>> approx_contours{best.approx};
    cv::drawContours(frame, approx_contours, 0, cv::Scalar(255, 0, 0), 2);
    cv::circle(
      frame,
      cv::Point(
        static_cast<int>(std::lround(best.center.x)),
        static_cast<int>(std::lround(best.center.y))),
      4,
      cv::Scalar(0, 0, 255),
      cv::FILLED);
  }

  void detectAprilTagAndPublish(cv::Mat & frame)
  {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, apriltag_dictionary_, corners, ids, apriltag_parameters_);

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

    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (apriltag_target_id_ >= 0 && ids[i] != apriltag_target_id_) {
        continue;
      }
      if (corners[i].size() < 4) {
        continue;
      }

      cv::Point2f center(0.0F, 0.0F);
      for (const auto & corner : corners[i]) {
        center += corner;
      }
      center *= 1.0F / static_cast<float>(corners[i].size());

      const double dx = static_cast<double>(center.x - image_center.x);
      const double dy = static_cast<double>(center.y - image_center.y);
      const double distance = std::hypot(dx, dy);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = static_cast<int>(i);
        best_center = center;
      }
    }

    if (best_index < 0) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "AprilTags found, but none matched target id %d.", apriltag_target_id_);
      return;
    }

    publishFineDataFromCenter(frame, best_center);
    cv::aruco::drawDetectedMarkers(frame, corners, ids);
    cv::circle(
      frame,
      cv::Point(
        static_cast<int>(std::lround(best_center.x)),
        static_cast<int>(std::lround(best_center.y))),
      4,
      cv::Scalar(0, 0, 255),
      cv::FILLED);
  }

  void frameTimerCallback()
  {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!camera_.isOpened()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Camera is not opened.");
        return;
      }

      if (!camera_.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to read frame from camera.");
        return;
      }
    }

    if (vision_target_mode_ == kVisionModeBlackSquare) {
      detectBlueSquareAndPublish(frame);
    } else if (vision_target_mode_ == kVisionModeAprilTag) {
      detectAprilTagAndPublish(frame);
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
  int blue_hue_min_;
  int blue_hue_max_;
  int blue_saturation_min_;
  int blue_value_min_;
  double min_square_area_;
  double min_square_fill_ratio_;
  std::string apriltag_dictionary_name_;
  int apriltag_target_id_;
  uint8_t vision_target_mode_;

  std::mutex frame_mutex_;
  cv::VideoCapture camera_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vision_mode_sub_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
  cv::Ptr<cv::aruco::Dictionary> apriltag_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> apriltag_parameters_;

  static constexpr uint8_t kVisionModeIdle = 0;
  static constexpr uint8_t kVisionModeBlackSquare = 1;
  static constexpr uint8_t kVisionModeAprilTag = 2;
  static constexpr double kMaxSquareAspectRatio = 1.25;
  static constexpr double kMaxSquareCornerCosine = 0.35;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
