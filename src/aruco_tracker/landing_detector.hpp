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

private:
  std::vector<MarkerDesc>               markers_;
  cv::Ptr<cv::aruco::Dictionary>        dictionary_;
  cv::Ptr<cv::aruco::Board>             board_;
  tf2::Vector3                          center_offset_{0,0,0};
};

/// ROS 2 node – detects landing pad, publishes TF + PoseStamped
class LandingDetectorNode : public rclcpp::Node {
public:
  explicit LandingDetectorNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

private:
  // callbacks
  void infoCB(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCB(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

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
};

} // namespace aruco_tracker