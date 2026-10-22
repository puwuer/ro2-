cmake_minimum_required(VERSION 3.8)
project(tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mycar_interfaces REQUIRED)
# 【必须添加】链接 ONNX Runtime 库
include_directories(/usr/include/onnxruntime)

add_executable(yolo_tracker_node src/yolo_tracker_node.cpp)
add_executable(yolo_tracker_node_unn src/yolo_tracker_node_unn.cpp)
ament_target_dependencies(yolo_tracker_node
  rclcpp
  image_transport
  cv_bridge
  message_filters
  sensor_msgs
  geometry_msgs
)
ament_target_dependencies(yolo_tracker_node_unn
  rclcpp
  image_transport
  cv_bridge
  message_filters
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  rclcpp_action
  mycar_interfaces
)
target_link_libraries(yolo_tracker_node ${OpenCV_LIBS})
install(TARGETS yolo_tracker_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS yolo_tracker_node_unn DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION share/${PROJECT_NAME} OPTIONAL)

# 将 ONNX Runtime 与 OpenCV 同时链接
target_link_libraries(yolo_tracker_node_unn 
  ${OpenCV_LIBS} 
  onnxruntime
)

# 将 find_path 的部分改为：
find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS /usr/include/onnxruntime /usr/local/include/onnxruntime
)
# 库的查找保持不变
find_library(ONNXRUNTIME_LIB
    NAMES onnxruntime
    PATHS /usr/lib /usr/local/lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
