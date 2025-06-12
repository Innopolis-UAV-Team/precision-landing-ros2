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

  // Declare and get history_size parameter
  declare_parameter("history_size", 1);
  get_parameter("history_size", history_size_);
  if (history_size_ < 1) {
	RCLCPP_WARN(get_logger(), "Invalid history size %zu, setting to 1", history_size_);
	history_size_ = 1;
  }
  RCLCPP_INFO(get_logger(), "History size for smoothing: %zu", history_size_);

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
    std::bind(&LandingDetectorNode::imageCB, this, std::placeholders::_1), "compressed");
  RCLCPP_INFO(get_logger(), "LandingDetectorNode initialized with config: %s", cfg.c_str());
  RCLCPP_INFO(get_logger(), "Map frame: %s, Landing pad frame: %s", map_frame_.c_str(), pad_frame_.c_str());
  RCLCPP_INFO(get_logger(), "Image topic: %s, Camera info topic: %s", 
	get_parameter("image_topic").as_string().c_str(), 
	get_parameter("camera_info_topic").as_string().c_str());


  position_history_.clear();
  rotation_history_.clear();

  initializeSmoothing();
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

		// Convert rvec to rotation matrix
		cv::Mat rotation_matrix;
		cv::Rodrigues(rvec_temp[0], rotation_matrix);

		// Transform the rotation matrix to match ROS conventions (example: swap axes)
		cv::Mat transform_matrix = (cv::Mat_<double>(3, 3) <<
				1,  0,  0,
				0, -1,  0,
				0,  0, -1); // Example transformation: flip Y and Z axes
		rotation_matrix = transform_matrix * rotation_matrix;

		// Convert back to rvec
		cv::Rodrigues(rotation_matrix, rvec_temp[0]);

		// Append results
		rvecs.insert(rvecs.end(), rvec_temp.begin(), rvec_temp.end());
		tvecs.insert(tvecs.end(), tvec_temp.begin(), tvec_temp.end());

	// Publish marker information
			RCLCPP_INFO(get_logger(), "Marker ID %d: rvec = [%f°, %f°, %f°], tvec = [%f, %f, %f]",
				ids[i], 
				rvec_temp[0][0] * 180.0 / M_PI, 
				rvec_temp[0][1] * 180.0 / M_PI, 
				rvec_temp[0][2] * 180.0 / M_PI,
				tvec_temp[0][0], 
				tvec_temp[0][1], 
				tvec_temp[0][2]);

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
		rvec[0] = normalize_angle(rvec[0]+M_PI);
		rvec[1] = normalize_angle(rvec[1]);
		rvec[2] = normalize_angle(-rvec[2]);
	}
	// Apply normalization to the weighted average rvec
	rvec[0] = normalize_angle(rvec[0]+M_PI);
	rvec[1] = normalize_angle(rvec[1]);
	rvec[2] = normalize_angle(-rvec[2]);

	// pose in map frame
	tf2::Transform map_cam_tf;

	try {

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

		// Найти маркер по ID
		auto marker_it = std::find_if(
			board_cfg_.getMarkers().begin(),
			board_cfg_.getMarkers().end(),
			[&](const MarkerDesc &marker) { return marker.id == ids[i]; });
		if (marker_it == board_cfg_.getMarkers().end()) {
			RCLCPP_WARN(get_logger(), "Marker ID %d not found in configuration", ids[i]);
			continue;
		}

				// Получить смещение по осям x и y
		double offset_x = marker_it->x;
		double offset_y = marker_it->y;
		// Вывести смещение в лог
		RCLCPP_INFO(get_logger(), "Marker ID %d: Offset x = %f, y = %f", ids[i], offset_x, offset_y);

		// publish marker offset from json
		geometry_msgs::msg::TransformStamped tf_marker_offset;
		tf_marker_offset.header.stamp = this->get_clock()->now();
		tf_marker_offset.header.frame_id = "marker_" + std::to_string(ids[i]);
		tf_marker_offset.child_frame_id = "marker_" + std::to_string(ids[i]) + "_offset";
		tf_marker_offset.transform.translation.x = -offset_x; // Example offset, replace with actual values from JSON
		tf_marker_offset.transform.translation.y = -offset_y;
		tf_marker_offset.transform.translation.z = 0.0;

		// turn roll on 180 degrees
		tf2::Quaternion q_offset;
		q_offset.setRPY(0, 0, 0); // Roll 180 degrees
		tf_marker_offset.transform.rotation = tf2::toMsg(q_offset);

		tf_broadcaster_->sendTransform(tf_marker_offset);
		RCLCPP_INFO(get_logger(), "Published TF for marker ID %d", ids[i]);


		for (size_t i = 0; i < ids.size(); ++i) {
		std::string marker_frame = "marker_" + std::to_string(ids[i]) + "_offset";
		if (transform_histories_.find(marker_frame) == transform_histories_.end()) {
			transform_histories_[marker_frame] = TransformHistory(history_size_);
			RCLCPP_INFO(get_logger(), "Initialized history for marker frame: %s", marker_frame.c_str());
		}
}
	}

}

void LandingDetectorNode::logAllKeys() {
    RCLCPP_INFO(get_logger(), "Listing all keys in transform_histories_:");
    for (const auto &entry : transform_histories_) {
        RCLCPP_INFO(get_logger(), "Key: %s", entry.first.c_str());
    }
}

void LandingDetectorNode::smoothAndPublishTF() {
    // logAllKeys();

    try {
        for (const auto &entry : transform_histories_) {
            const std::string &marker_frame = entry.first;

            if (!tf_buffer_->canTransform(map_frame_, marker_frame, tf2::TimePointZero)) {
                continue;
            }

            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(map_frame_, marker_frame, tf2::TimePointZero);

            tf2::Transform tf_current;
            tf2::fromMsg(transform.transform, tf_current);

            // Добавляем текущую трансформацию в историю
            transform_histories_[marker_frame].add(tf_current);

            // Получаем сглаженную трансформацию
            tf2::Transform tf_smoothed = transform_histories_[marker_frame].getSmoothedTransform();

            // Публикуем сглаженную трансформацию
            geometry_msgs::msg::TransformStamped smoothed_transform;
            smoothed_transform.header.stamp = this->get_clock()->now();
            smoothed_transform.header.frame_id = map_frame_;
            smoothed_transform.child_frame_id = pad_frame_;
            smoothed_transform.transform = tf2::toMsg(tf_smoothed);
            tf_broadcaster_->sendTransform(smoothed_transform);

			// Publish PoseStamped message
			geometry_msgs::msg::PoseStamped pose_msg;
			pose_msg.header = smoothed_transform.header;
			pose_msg.pose.position.x = tf_smoothed.getOrigin().x();
			pose_msg.pose.position.y = tf_smoothed.getOrigin().y();
			pose_msg.pose.position.z = tf_smoothed.getOrigin().z();
			tf2::Quaternion q = tf_smoothed.getRotation();
			pose_msg.pose.orientation.x = q.x();
			pose_msg.pose.orientation.y = q.y();
			pose_msg.pose.orientation.z = q.z();
			pose_msg.pose.orientation.w = q.w();

			// Publish the PoseStamped message
			pose_pub_->publish(pose_msg);

        }
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
    }
}

    void LandingDetectorNode::initializeSmoothing() {
        timer_ = create_wall_timer(
            std::chrono::milliseconds(30), // pfs: 30 ms
            std::bind(&LandingDetectorNode::smoothAndPublishTF, this));
    }

} // namespace aruco_tracker

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_tracker::LandingDetectorNode>());
  rclcpp::shutdown();
  return 0;
}