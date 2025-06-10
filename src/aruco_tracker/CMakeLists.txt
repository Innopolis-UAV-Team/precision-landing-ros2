cmake_minimum_required(VERSION 3.5)
project(aruco_tracker)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# NOTE: we build opencv from source, hence the path /usr/local/lib
find_package(OpenCV 4.10.0 REQUIRED PATHS /usr/local NO_DEFAULT_PATH)

find_package(yaml-cpp REQUIRED)
find_package(image_transport REQUIRED)
# Add the path to nlohmann_json if necessary
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH} /usr/lib/cmake/nlohmann_json")
find_package(nlohmann_json REQUIRED)


include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(/usr/local/lib)

add_executable(aruco_tracker ArucoTracker.cpp)
add_executable(landing_detector_node landing_detector_node.cpp)
ament_target_dependencies(landing_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(landing_detector_node
  ${OpenCV_LIBS}
  yaml-cpp
  nlohmann_json
)

install(TARGETS landing_detector_node
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(aruco_tracker ${OpenCV_LIBS})

ament_target_dependencies(aruco_tracker
  Eigen3
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_compile_features(aruco_tracker PUBLIC c_std_99 cxx_std_17)

install(TARGETS aruco_tracker DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)
ament_package()
