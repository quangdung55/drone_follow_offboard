cmake_minimum_required(VERSION 3.8)
project(drone_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ardupilot_msgs REQUIRED)

# ==============================================================================
# 1. CORE LIBRARY (Thuật toán thuần: Control, Estimator, Safety, Math)
# ==============================================================================
add_library(${PROJECT_NAME}_core_lib SHARED
  src/core/control/ap_control.cpp
  src/core/control/ap_follow.cpp
  src/core/control/follow_controller.cpp
  src/core/control/yaw_controller.cpp
  src/core/estimator/target_estimator.cpp
  src/core/math/JitterCorrection.cpp
  src/core/safety/safety.cpp
)

target_include_directories(${PROJECT_NAME}_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Thư viện Core CHỈ phụ thuộc vào Eigen (Toán học), tuyệt đối không có ROS
target_link_libraries(${PROJECT_NAME}_core_lib PUBLIC Eigen3::Eigen)

# ==============================================================================
# 2. ROS 2 WRAPPER LIBRARY (Lớp vỏ bọc giao tiếp ROS)
# ==============================================================================
add_library(${PROJECT_NAME}_wrapper_lib SHARED
  # src/ros2/camera_to_gps_node.cpp
  src/ros2/smart_follow_node.cpp
)

target_include_directories(${PROJECT_NAME}_wrapper_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Liên kết Wrapper với Core Lib
target_link_libraries(${PROJECT_NAME}_wrapper_lib ${PROJECT_NAME}_core_lib)

# Liên kết Wrapper với hệ sinh thái ROS 2
ament_target_dependencies(${PROJECT_NAME}_wrapper_lib
  rclcpp sensor_msgs geometry_msgs geographic_msgs
  std_msgs diagnostic_msgs tf2 tf2_geometry_msgs ardupilot_msgs
)

# ==============================================================================
# 3. EXECUTABLES (Điểm khởi chạy)
# ==============================================================================
add_executable(smart_follow_node src/smart_follow_main.cpp)
target_link_libraries(smart_follow_node ${PROJECT_NAME}_wrapper_lib)

# ==============================================================================
# 4. INSTALL & EXPORT
# ==============================================================================
install(TARGETS smart_follow_node
  DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp sensor_msgs geometry_msgs geographic_msgs
  std_msgs diagnostic_msgs tf2 tf2_geometry_msgs Eigen3
)

install(TARGETS ${PROJECT_NAME}_core_lib ${PROJECT_NAME}_wrapper_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ==============================================================================
# 5. UNIT TESTS (Kiểm thử độc lập)
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  ament_add_gtest(test_follow_controller test/core/test_follow_controller_example.cpp)
  target_link_libraries(test_follow_controller ${PROJECT_NAME}_core_lib)
  
  ament_add_gtest(test_smart_follow_node test/ros2/test_smart_follow_node.cpp)
  target_link_libraries(test_smart_follow_node ${PROJECT_NAME}_wrapper_lib)
endif()

ament_package()