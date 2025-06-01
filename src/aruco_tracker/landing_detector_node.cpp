#include "landing_detector.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <unordered_map>

namespace aruco_tracker {

// ------------------------- BoardConfig implementation -------------------------
void BoardConfig::load(const std::string &path)
{
  YAML::Node root = YAML::LoadFile(path);

  static const std::unordered_map<std::string,int> dict_map = {
    {"DICT_4X4_50",         cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100",        cv::aruco::DICT_4X4_100},
    {"DICT_5X5_50",         cv::aruco::DICT_5X5_50},
    {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11}
  };

  std::string dict_name = root[0]["type"].as<std::string>("Aruco_4x4_50");
  if (dict_name == "Aruco_4x4_50") dict_name = "DICT_4X4_50";
  else if (dict_name == "Aruco_4x4_100") dict_name = "DICT_4X4_100";
  else if (dict_name == "Aruco_5x5_50") dict_name = "DICT_5X5_50";
  else if (dict_name == "AprilTag_36h11") dict_name = "DICT_APRILTAG_36h11";
  auto it = dict_map.find(dict_name);
  if (it == dict_map.end())
    throw std::runtime_error("Unknown ArUco dictionary: " + dict_name);
  dictionary_ = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(it->second));

  std::vector<int> ids;
  std::vector<std::vector<cv::Point3f>> objPoints;

  for (const auto &m : root)
  {
    MarkerDesc md;
    md.id   = m["id"].as<int>();
    auto corners = m["corners"];
    if (!corners || corners.size() != 4)
      throw std::runtime_error("Each marker must have 4 corners");

    std::vector<cv::Point3f> pts;
    for (const auto &pt : corners) {
      pts.emplace_back(pt["x"].as<float>(), pt["y"].as<float>(), pt["z"].as<float>());
    }

    // estimate centre + size
    float cx = 0, cy = 0;
    for (const auto &p : pts) { cx += p.x; cy += p.y; }
    cx /= 4; cy /= 4;
    float s = std::hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
    md.x = cx; md.y = cy; md.size = s; md.yaw = 0;
    markers_.push_back(md);

    ids.push_back(md.id);
    objPoints.push_back(pts);
  }

  board_ = cv::makePtr<cv::aruco::Board>(objPoints, *dictionary_, ids);
  center_offset_.setValue(0, 0, 0);
}


// ---------------------- LandingDetectorNode implementation --------------------
LandingDetectorNode::LandingDetectorNode(const rclcpp::NodeOptions &opts)
: Node("landing_detector", opts)
{
  // declare & get parameters
  declare_parameter("config_path",       "board.yml");
  declare_parameter("map_frame",         "map");
  declare_parameter("landing_pad_frame", "landing_pad");
  declare_parameter("image_topic",       "/camera/image_raw");
  declare_parameter("camera_info_topic", "/camera/camera_info");
  declare_parameter("invert",            false);

  get_parameter("map_frame",         map_frame_);
  get_parameter("landing_pad_frame", pad_frame_);
  get_parameter("invert",            invert_image_);
  get_parameter("landing_pad_frame", pad_frame_);

  std::string cfg; get_parameter("config_path", cfg);
  board_cfg_.load(cfg);

  // TF helpers
  tf_buffer_      = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // publishers
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("landing_pad/pose", 10);

  debug_image_pub_ = image_transport::create_publisher(this, "debug_image");

  // subscribers
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    get_parameter("camera_info_topic").as_string(), 10,
    std::bind(&LandingDetectorNode::infoCB, this, std::placeholders::_1));

  image_sub_ = image_transport::create_subscription(
    this, get_parameter("image_topic").as_string(),
    std::bind(&LandingDetectorNode::imageCB, this, std::placeholders::_1), "raw");
}

void LandingDetectorNode::infoCB(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  camera_frame_ = msg->header.frame_id;
  RCLCPP_INFO(get_logger(), "[camera_info] frame_id = %s", camera_frame_.c_str());
  cam_matrix_ = cv::Mat(3,3,CV_64F,const_cast<double*>(msg->k.data())).clone();
  dist_coeffs_= cv::Mat(msg->d.size(),1,CV_64F,const_cast<double*>(msg->d.data())).clone();
  has_camera_info_ = true;
  camera_info_sub_.reset();
  RCLCPP_INFO(get_logger(), "Camera info received");
}

void LandingDetectorNode::imageCB(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  if (!has_camera_info_) return;

  cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  cv::Mat frame_out = frame.clone();
  if (invert_image_) {
    cv::bitwise_not(frame, frame);
    RCLCPP_INFO(get_logger(), "Image inverted due to 'invert' param");
  }

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(frame, board_cfg_.dictionary(), corners, ids);
  RCLCPP_INFO(get_logger(), "Detected %lu markers", ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    const auto &c = corners[i];
    RCLCPP_INFO(get_logger(), " - ID %d: [%f,%f] [%f,%f] [%f,%f] [%f,%f]",
      ids[i],
      c[0].x, c[0].y, c[1].x, c[1].y, c[2].x, c[2].y, c[3].x, c[3].y);
  }

  // Draw markers on the original (non-inverted) frame
  cv::aruco::drawDetectedMarkers(frame_out, corners, ids);

  if (ids.empty()) {
  RCLCPP_WARN(get_logger(), "No board markers detected in image, skipping pose estimation.");
  debug_image_pub_.publish(cv_bridge::CvImage(
    msg->header, "bgr8", frame_out).toImageMsg());
  return;
}

cv::Vec3d rvec, tvec;
int valid = cv::aruco::estimatePoseBoard(corners, ids, board_cfg_.board(), cam_matrix_, dist_coeffs_, rvec, tvec);
  if (valid <= 0) return;

  geometry_msgs::msg::TransformStamped tf_cam_board;
  tf_cam_board.header.stamp = this->get_clock()->now();
  tf_cam_board.header.frame_id = camera_frame_;
  tf_cam_board.child_frame_id  = pad_frame_ + "_board";
  tf_cam_board.transform.translation.x = tvec[0];
  tf_cam_board.transform.translation.y = tvec[1];
  tf_cam_board.transform.translation.z = tvec[2];
  tf2::Quaternion q; q.setRPY(rvec[0], rvec[1], rvec[2]);
  tf_cam_board.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(tf_cam_board);

  // shift to true centre
  tf2::Transform cam_board_tf, board_pad_tf;
  tf2::fromMsg(tf_cam_board.transform, cam_board_tf);
  board_pad_tf.setOrigin(board_cfg_.centerOffset());
  board_pad_tf.setRotation(tf2::Quaternion::getIdentity());
  tf2::Transform cam_pad_tf = cam_board_tf * board_pad_tf;

  geometry_msgs::msg::TransformStamped tf_cam_pad;
  tf_cam_pad.header.stamp = this->get_clock()->now();
  tf_cam_pad.header.frame_id = camera_frame_;
  tf_cam_pad.child_frame_id = pad_frame_;
  tf_cam_pad.transform = tf2::toMsg(cam_pad_tf);
  tf_broadcaster_->sendTransform(tf_cam_pad);

  // pose in map frame
  try {
    auto map_cam = tf_buffer_->lookupTransform(
      map_frame_, camera_frame_, tf_cam_pad.header.stamp, tf2::durationFromSec(0.03));

    tf2::Transform map_cam_tf, cam_pad_tf2;
    tf2::fromMsg(map_cam.transform, map_cam_tf);
    tf2::fromMsg(tf_cam_pad.transform, cam_pad_tf2);

    tf2::Transform map_pad_tf = map_cam_tf * cam_pad_tf2;

    geometry_msgs::msg::PoseStamped pad_pose;
    pad_pose.header = tf_cam_pad.header;
    pad_pose.header.frame_id = map_frame_;
    pad_pose.pose.position.x = map_pad_tf.getOrigin().x();
    pad_pose.pose.position.y = map_pad_tf.getOrigin().y();
    pad_pose.pose.position.z = map_pad_tf.getOrigin().z();
    pad_pose.pose.orientation = tf2::toMsg(map_pad_tf.getRotation());
    pose_pub_->publish(pad_pose);

// publish debug image
debug_image_pub_.publish(cv_bridge::CvImage(
  msg->header, "bgr8", frame_out).toImageMsg());
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
  }
}

} // namespace aruco_tracker

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_tracker::LandingDetectorNode>());
  rclcpp::shutdown();
  return 0;
}