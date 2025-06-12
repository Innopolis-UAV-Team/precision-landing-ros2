#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp> // Include JSON library
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>

namespace aruco_tracker {

struct MarkerDesc {
  int id;
  double size;               ///< side length [m]
  double x;
  double y;
  double yaw;                ///< rotation around Z [rad]
};

/**
 * @brief Parses board YAML/JSON and builds an OpenCV aruco::Board.
 * YAML keys:
 *   dictionary: DICT_4X4_50 | DICT_APRILTAG_36h11 | ...
 *   offset_x/y/z [m] – pad‑centre shift (optional)
 *   markers: [ {id,size,x,y,yaw}, ... ]
 */
class BoardConfig {
public:
	void load(const std::string &path);          ///< throws on error
	const cv::Ptr<cv::aruco::Board>      &board()      const { return board_; }
	const cv::Ptr<cv::aruco::Dictionary> &dictionary() const { return dictionary_; }
	const tf2::Vector3                   &centerOffset()const { return center_offset_; }
	const std::unordered_map<int, double>& getMarkerSizes() const { return marker_sizes_; } // Public getter
	const std::vector<MarkerDesc>& getMarkers() const { return markers_; } // Public getter for markers_

private:
	std::vector<MarkerDesc>               markers_;
	cv::Ptr<cv::aruco::Dictionary>        dictionary_;
	cv::Ptr<cv::aruco::Board>             board_;
	tf2::Vector3                          center_offset_{0,0,0};
	std::unordered_map<int, double>       marker_sizes_; // Map marker IDs to their sizes
};

// Класс для хранения истории трансформаций и вычисления сглаженной трансформации
class TransformHistory {
public:
    TransformHistory(size_t max_size = 10) : max_size_(max_size) {}

    void add(const tf2::Transform &transform) {
        if (history_.size() >= max_size_) {
            history_.pop_front();
        }
        history_.push_back(transform);
    }

    tf2::Transform getSmoothedTransform() const {
        tf2::Vector3 translation(0, 0, 0);
        tf2::Quaternion rotation(0, 0, 0, 0);

        for (const auto &tf : history_) {
            translation += tf.getOrigin();
            rotation += tf.getRotation();
        }

        translation /= history_.size();
        rotation.normalize();

        tf2::Transform smoothed_transform;
        smoothed_transform.setOrigin(translation);
        smoothed_transform.setRotation(rotation);

        return smoothed_transform;
    }

    void setMaxSize(size_t max_size) {
        max_size_ = max_size;
    }

private:
    std::deque<tf2::Transform> history_;
    size_t max_size_;
};

/// ROS 2 node – detects landing pad, publishes TF + PoseStamped
class LandingDetectorNode : public rclcpp::Node {
public:
  explicit LandingDetectorNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

private:
  // callbacks
  void infoCB(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCB(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void logAllKeys(); // For debugging purposes, logs all keys in transform_histories_
  void publishMarkerTF(const std::vector<int> &ids, const std::vector<std::vector<cv::Point2f>> &corners);

  
  // parameters & config
  bool invert_image_ = false;
  BoardConfig board_cfg_;
  std::string camera_frame_, map_frame_, pad_frame_;

  // intrinsics
  cv::Mat cam_matrix_, dist_coeffs_;
  bool has_camera_info_ = false;

  // ROS interfaces
  image_transport::Publisher debug_image_pub_;
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::deque<cv::Vec3d> position_history_;
  std::deque<cv::Vec3d> rotation_history_;
  size_t history_size_ = 10; // Sliding window size for median filter

  // Smoothing and Transform History
  void initializeSmoothing(); // Объявление метода для инициализации сглаживания
  void smoothAndPublishTF();  // Объявление метода для сглаживания и публикации TF

  rclcpp::TimerBase::SharedPtr timer_; // Таймер для периодического вызова smoothAndPublishTF
  std::unordered_map<std::string, TransformHistory> transform_histories_; // История трансформаций
};

} // namespace aruco_tracker