#include "landing_detector.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <unordered_map>
#include <algorithm>
#include <deque>
#include <fstream>
#include <nlohmann/json.hpp>


static const std::unordered_map<std::string, int> dict_map = {
    {"Aruco_4x4_50",        cv::aruco::DICT_4X4_50},
    {"DICT_4X4_50",         cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100",        cv::aruco::DICT_4X4_100},
    {"DICT_5X5_50",         cv::aruco::DICT_5X5_50},
    {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
};
namespace aruco_tracker {

// ------------------------- BoardConfig implementation -------------------------
void BoardConfig::load(const std::string &path)
{
	std::ifstream file(path);
	if (!file.is_open()) {
		throw std::runtime_error("Failed to open board configuration file: " + path);
	}

	nlohmann::json root;
	file >> root;

	static const std::unordered_map<std::string, int> dict_map = {
		{"Aruco_4x4_50",        cv::aruco::DICT_4X4_50},
		{"DICT_4X4_50",         cv::aruco::DICT_4X4_50},
		{"DICT_4X4_100",        cv::aruco::DICT_4X4_100},
		{"DICT_5X5_50",         cv::aruco::DICT_5X5_50},
		{"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
		{"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
	};

	// Validate global fields
	if (!root.contains("sizeX") || !root.contains("sizeY") || !root.contains("pixelCountPerMeter") || !root.contains("colorInverted")) {
		throw std::runtime_error("Invalid JSON file: missing global fields.");
	}

	// Validate markers array
	if (!root.contains("markers") || !root["markers"].is_array() || root["markers"].empty()) {
		throw std::runtime_error("Invalid JSON file: missing or empty 'markers' field.");
	}

	// Parse dictionary type
	std::string dict_name = root["markers"][0]["type"];
	auto it = dict_map.find(dict_name);
	if (it == dict_map.end()) {
		throw std::runtime_error("Unknown ArUco dictionary: " + dict_name);
	}
	dictionary_ = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(it->second));

	std::vector<int> ids;
	std::vector<std::vector<cv::Point3f>> objPoints;

	for (const auto &m : root["markers"]) {
		// Validate marker fields
		if (!m.contains("id") || !m.contains("x") || !m.contains("y") || !m.contains("size") || !m.contains("yaw")) {
			throw std::runtime_error("Invalid marker definition in JSON file.");
		}

		MarkerDesc md;
		md.id = m["id"];
		md.x = m["x"];
		md.y = m["y"];
		md.size = m["size"];
		md.yaw = m["yaw"];
		markers_.push_back(md);

		std::vector<cv::Point3f> pts;
		float half_size = md.size / 2.0;
		pts.emplace_back(md.x - half_size, md.y + half_size, 0);
		pts.emplace_back(md.x + half_size, md.y + half_size, 0);
		pts.emplace_back(md.x + half_size, md.y - half_size, 0);
		pts.emplace_back(md.x - half_size, md.y - half_size, 0);

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

  
  std::string cfg; get_parameter("config_path", cfg);
  board_cfg_.load(cfg);

  // TF helpers
  tf_buffer_      = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // publishers
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/landing_target/pose", 10);

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
  cv::Mat frame_proc = frame.clone(); // This will be used for processing (may be inverted)
  if (invert_image_) {
    cv::bitwise_not(frame_proc, frame_proc);
    RCLCPP_INFO(get_logger(), "Image inverted due to 'invert' param");
  }
  std::vector<int> ids; std::vector<std::vector<cv::Point2f>> corners;

	// Create DetectorParameters using cv::makePtr
	cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::makePtr<cv::aruco::DetectorParameters>();
	detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

	// Detect markers with refined corners
	cv::aruco::detectMarkers(frame_proc, board_cfg_.dictionary(), corners, ids, detector_params);
  RCLCPP_INFO(get_logger(), "Detected %lu markers", ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    const auto &c = corners[i];
    RCLCPP_INFO(get_logger(), " - ID %d: [%f,%f] [%f,%f] [%f,%f] [%f,%f]",
      ids[i],
      c[0].x, c[0].y, c[1].x, c[1].y, c[2].x, c[2].y, c[3].x, c[3].y);
  }

  // Draw markers for debug image (always on original, non-inverted frame)
  cv::Mat frame_out = frame.clone();
  cv::aruco::drawDetectedMarkers(frame_out, corners, ids);

  if (ids.empty()) {
    RCLCPP_WARN(get_logger(), "No board markers detected in image, skipping pose estimation.");
    // Ensure the debug image is published correctly
    cv_bridge::CvImage debug_msg;
    debug_msg.header = msg->header;
    debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
    debug_msg.image = frame_out; // Use the original frame for debug output
    debug_image_pub_.publish(debug_msg.toImageMsg());
    return;
  }

  cv::Vec3d rvec, tvec;
  std::vector<cv::Vec3d> rvecs, tvecs;
  std::vector<std::vector<cv::Point2f>> single_corners; // Prepare corners for each marker

  for (size_t i = 0; i < ids.size(); ++i) {
		// Retrieve marker size from the configuration using the public getter
		auto marker_it = std::find_if(board_cfg_.getMarkers().begin(), board_cfg_.getMarkers().end(),
			[&](const MarkerDesc &marker) { return marker.id == ids[i]; });

		if (marker_it == board_cfg_.getMarkers().end()) {
			RCLCPP_WARN(get_logger(), "Marker size not found for ID %d", ids[i]);
			continue;
		}

		double marker_size = marker_it->size;

		// Add corners for the current marker
		single_corners.push_back(corners[i]);

		// Use ArUco's estimatePoseSingleMarkers for pose estimation
		std::vector<cv::Vec3d> rvec_temp, tvec_temp;
		cv::aruco::estimatePoseSingleMarkers(single_corners, marker_size, cam_matrix_, dist_coeffs_, rvec_temp, tvec_temp);

		// Append results
		rvecs.insert(rvecs.end(), rvec_temp.begin(), rvec_temp.end());
		tvecs.insert(tvecs.end(), tvec_temp.begin(), tvec_temp.end());

		// Publish marker information
		RCLCPP_INFO(get_logger(), "Marker ID %d: rvec = [%f, %f, %f], tvec = [%f, %f, %f]",
			ids[i], rvec_temp[0][0], rvec_temp[0][1], rvec_temp[0][2],
			tvec_temp[0][0], tvec_temp[0][1], tvec_temp[0][2]);

		// Clear single_corners for the next marker
		single_corners.clear();
	}

	if (rvecs.empty()) {
		RCLCPP_WARN(get_logger(), "Pose estimation failed: no valid markers");
		cv_bridge::CvImage debug_msg;
		debug_msg.header = msg->header;
		debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
		debug_msg.image = frame_out; // Use the original frame for debug output
		debug_image_pub_.publish(debug_msg.toImageMsg());
		return;
	}

	// Publish debug image after processing
	cv_bridge::CvImage debug_msg;
	debug_msg.header = msg->header;
	debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
	debug_msg.image = frame_out; // Use the processed frame for debug output
	debug_image_pub_.publish(debug_msg.toImageMsg());

	// Apply weighted average filter for smoothing angles
	cv::Vec3d rvec_weighted{0, 0, 0}, tvec_weighted{0, 0, 0};
	double total_weight = 0.0;

	for (size_t i = 0; i < rvecs.size(); ++i) {
		double weight = 1.0 / (cv::norm(tvecs[i]) + 1e-6); // Weight inversely proportional to distance
		rvec_weighted += weight * rvecs[i];
		tvec_weighted += weight * tvecs[i];
		total_weight += weight;
	}

	rvec = rvec_weighted / total_weight;
	tvec = tvec_weighted / total_weight;

	// Normalize angles to the range [-π, π]
	auto normalize_angle = [](double angle) -> double {
		while (angle > M_PI) angle -= 2.0 * M_PI;
		while (angle < -M_PI) angle += 2.0 * M_PI;
		return angle;
	};

	// Apply normalization to rvecs
	for (auto &rvec : rvecs) {
		rvec[0] = normalize_angle(rvec[0]);
		rvec[1] = normalize_angle(rvec[1]);
		rvec[2] = normalize_angle(rvec[2]);
	}

	// Apply normalization to the weighted average rvec
	rvec[0] = normalize_angle(rvec[0]);
	rvec[1] = normalize_angle(rvec[1]);
	rvec[2] = normalize_angle(rvec[2]);

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
	tf2::Transform map_cam_tf;

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

		// publish debug image (always non-inverted)
		debug_image_pub_.publish(cv_bridge::CvImage(
			msg->header, "bgr8", frame_out).toImageMsg());
	} catch (const tf2::TransformException &ex) {
		RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
		// publish debug image even if TF fails
		debug_image_pub_.publish(cv_bridge::CvImage(
			msg->header, "bgr8", frame_out).toImageMsg());
	}

	for (size_t i = 0; i < ids.size(); ++i) {
		// Publish TF for each detected marker
		geometry_msgs::msg::TransformStamped tf_marker;
		tf_marker.header.stamp = this->get_clock()->now();
		tf_marker.header.frame_id = camera_frame_;
		tf_marker.child_frame_id = "marker_" + std::to_string(ids[i]);
		tf_marker.transform.translation.x = tvecs[i][0];
		tf_marker.transform.translation.y = tvecs[i][1];
		tf_marker.transform.translation.z = tvecs[i][2];
		tf2::Quaternion q; q.setRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]);
		tf_marker.transform.rotation = tf2::toMsg(q);
		tf_broadcaster_->sendTransform(tf_marker);
	}

	// Median filter with sliding average
	position_history_.push_back(tvec);
	rotation_history_.push_back(rvec);

	if (position_history_.size() > history_size_) {
		position_history_.pop_front();
		rotation_history_.pop_front();
	}

	std::vector<cv::Vec3d> sorted_positions(position_history_.begin(), position_history_.end());
	std::vector<cv::Vec3d> sorted_rotations(rotation_history_.begin(), rotation_history_.end());

	std::sort(sorted_positions.begin(), sorted_positions.end(), [](const cv::Vec3d &a, const cv::Vec3d &b) {
		return cv::norm(a) < cv::norm(b);
	});
	std::sort(sorted_rotations.begin(), sorted_rotations.end(), [](const cv::Vec3d &a, const cv::Vec3d &b) {
		return cv::norm(a) < cv::norm(b);
	});

	cv::Vec3d median_position = sorted_positions[sorted_positions.size() / 2];
	cv::Vec3d median_rotation = sorted_rotations[sorted_rotations.size() / 2];

	// Publish filtered coordinates in landing_pad_frame
	geometry_msgs::msg::TransformStamped tf_filtered;
	tf_filtered.header.stamp = this->get_clock()->now();
	tf_filtered.header.frame_id = camera_frame_;
	tf_filtered.child_frame_id = pad_frame_;
	tf_filtered.transform.translation.x = median_position[0];
	tf_filtered.transform.translation.y = median_position[1];
	tf_filtered.transform.translation.z = median_position[2];
	tf2::Quaternion q_filtered; q_filtered.setRPY(median_rotation[0], median_rotation[1], median_rotation[2]);
	tf_filtered.transform.rotation = tf2::toMsg(q_filtered);
	tf_broadcaster_->sendTransform(tf_filtered);
}

} // namespace aruco_tracker

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_tracker::LandingDetectorNode>());
  rclcpp::shutdown();
  return 0;
}