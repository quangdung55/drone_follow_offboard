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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Create library for shared code
add_library(smart_follow_lib SHARED
  src/smart_follow_node.cpp
  src/ap_control.cpp
  src/ap_follow.cpp
)

ament_target_dependencies(smart_follow_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  geographic_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(smart_follow_lib
  Eigen3::Eigen
)

# Create executable
add_executable(smart_follow_node src/smart_follow_main.cpp)
target_link_libraries(smart_follow_node smart_follow_lib)

ament_target_dependencies(smart_follow_node
  rclcpp
  sensor_msgs
  geometry_msgs
  geographic_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS smart_follow_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY include/
  DESTINATION include)

# Export library for downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  geographic_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)

# Install library with export
install(TARGETS smart_follow_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests
  ament_add_gtest(test_smart_follow_node
    test/test_smart_follow_node.cpp
  )
  
  target_link_libraries(test_smart_follow_node
    smart_follow_lib
    Eigen3::Eigen
  )
  
  target_include_directories(test_smart_follow_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
  )
  
  ament_target_dependencies(test_smart_follow_node
    rclcpp
    sensor_msgs
    geometry_msgs
    geographic_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
  )
endif()

ament_package()