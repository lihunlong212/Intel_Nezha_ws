#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

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
    black_threshold_(declare_parameter<int>("black_threshold", 31)),
    min_circle_area_(declare_parameter<double>("min_circle_area", 10340.0)),
    min_circularity_(declare_parameter<double>("min_circularity", 0.45))
  {
    fine_data_pub_ =
      create_publisher<std_msgs::msg::Int32MultiArray>(fine_data_topic_, rclcpp::QoS(10));

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
      "Camera node started. camera_device=%s show_preview=%s fine_data_topic=%s black_threshold=%d min_circle_area=%.1f min_circularity=%.2f",
      camera_device_.c_str(),
      show_preview_ ? "true" : "false",
      fine_data_topic_.c_str(),
      black_threshold_,
      min_circle_area_,
      min_circularity_);
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
  struct CircleCandidate
  {
    int contour_index{-1};
    cv::Point2f center{};
    float radius{0.0F};
    double area{0.0};
    double score{0.0};
  };

  void detectBlackCircleAndPublish(cv::Mat & frame)
  {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);

    cv::Mat value = hsv_channels[2];
    cv::GaussianBlur(value, value, cv::Size(5, 5), 0.0);

    cv::Mat mask;
    cv::Mat otsu_mask;
    const int fixed_threshold = std::max(0, std::min(black_threshold_, 255));
    const double otsu_threshold =
      cv::threshold(value, otsu_mask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    const int adaptive_threshold = kUseOtsu ?
      std::max(fixed_threshold, std::min(160, static_cast<int>(std::lround(otsu_threshold)))) :
      fixed_threshold;
    cv::threshold(value, mask, adaptive_threshold, 255, cv::THRESH_BINARY_INV);

    const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    const cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    CircleCandidate best;
    const double min_area = std::max(20.0, min_circle_area_);
    const double min_circularity = std::max(0.20, std::min(min_circularity_, 1.0));
    const double max_aspect_ratio = std::max(1.0, kMaxAspectRatio);
    const double min_fill_ratio = std::max(0.05, std::min(kMinFillRatio, 1.0));
    const double max_radial_error = std::max(0.01, kMaxRadialError);
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

      const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
      std::vector<cv::Point> approx;
      cv::approxPolyDP(contours[i], approx, 0.02 * perimeter, true);
      if (static_cast<int>(approx.size()) < std::max(4, kMinVertices)) {
        continue;
      }

      cv::Point2f center;
      float radius = 0.0F;
      cv::minEnclosingCircle(contours[i], center, radius);
      if (radius <= 0.0F) {
        continue;
      }

      const double circle_area = CV_PI * static_cast<double>(radius) * static_cast<double>(radius);
      const double fill_ratio = area / circle_area;
      if (fill_ratio < min_fill_ratio || fill_ratio > 1.15) {
        continue;
      }

      if (circularity < min_circularity && fill_ratio < 0.58) {
        continue;
      }

      double radial_error_sum = 0.0;
      for (const auto & point : contours[i]) {
        const double dx = static_cast<double>(point.x) - center.x;
        const double dy = static_cast<double>(point.y) - center.y;
        const double distance = std::sqrt(dx * dx + dy * dy);
        radial_error_sum += std::abs(distance - radius) / radius;
      }

      const double radial_error = radial_error_sum / static_cast<double>(contours[i].size());
      if (radial_error > max_radial_error) {
        continue;
      }

      const double aspect_score = 1.0 / aspect_ratio;
      const double fill_score = 1.0 - std::min(1.0, std::abs(0.85 - fill_ratio));
      const double radial_score = 1.0 - std::min(1.0, radial_error / max_radial_error);
      const double shape_score =
        0.35 * circularity + 0.25 * fill_score + 0.20 * aspect_score + 0.20 * radial_score;
      const double score = area * shape_score;
      if (score > best.score) {
        best.contour_index = static_cast<int>(i);
        best.center = center;
        best.radius = radius;
        best.area = area;
        best.score = score;
      }
    }

    if (best.contour_index < 0) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No black circle found. threshold=%d contours=%zu",
        adaptive_threshold, contours.size());
      return;
    }

    const cv::Moments moments = cv::moments(contours[static_cast<std::size_t>(best.contour_index)]);
    if (std::abs(moments.m00) <= std::numeric_limits<double>::epsilon()) {
      return;
    }

    const float circle_center_x = static_cast<float>(moments.m10 / moments.m00);
    const float circle_center_y = static_cast<float>(moments.m01 / moments.m00);
    const float image_center_x = static_cast<float>(frame.cols) / 2.0F;
    const float image_center_y = static_cast<float>(frame.rows) / 2.0F;

    const int x_offset = static_cast<int>(std::lround(image_center_y - circle_center_y));
    const int y_offset = static_cast<int>(std::lround(image_center_x - circle_center_x));

    std_msgs::msg::Int32MultiArray fine_data_msg;
    fine_data_msg.data = {x_offset, y_offset};
    fine_data_pub_->publish(fine_data_msg);

    cv::drawContours(frame, contours, best.contour_index, cv::Scalar(0, 255, 0), 2);
    cv::circle(
      frame,
      cv::Point(
        static_cast<int>(std::lround(best.center.x)),
        static_cast<int>(std::lround(best.center.y))),
      static_cast<int>(std::lround(best.radius)),
      cv::Scalar(255, 0, 0),
      2);
    cv::circle(
      frame,
      cv::Point(
        static_cast<int>(std::lround(circle_center_x)),
        static_cast<int>(std::lround(circle_center_y))),
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

    detectBlackCircleAndPublish(frame);

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
  int black_threshold_;
  double min_circle_area_;
  double min_circularity_;

  std::mutex frame_mutex_;
  cv::VideoCapture camera_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  rclcpp::TimerBase::SharedPtr frame_timer_;

  static constexpr double kMaxAspectRatio = 1.30;
  static constexpr double kMinFillRatio = 0.70;
  static constexpr int kMinVertices = 8;
  static constexpr double kMaxRadialError = 0.12;
  static constexpr bool kUseOtsu = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
