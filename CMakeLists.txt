cmake_minimum_required(VERSION 3.8)
project(vio_ros_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)  # ⭐ Added for camera frustum
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Force system spdlog to avoid conflicts (same as lidar_odometry_ros_wrapper)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SPDLOG REQUIRED spdlog)

# Build Ceres from thirdparty (needed for lightweight_vio)
set(LIGHTWEIGHT_VIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lightweight_vio)
set(THIRDPARTY_DIR ${LIGHTWEIGHT_VIO_DIR}/thirdparty)
set(CERES_DIR ${THIRDPARTY_DIR}/ceres-solver)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build Ceres examples" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "Build Ceres benchmarks" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build Ceres tests" FORCE)
set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "Disable Ceres uninstall target" FORCE)
set(SUITESPARSE OFF CACHE BOOL "Disable SuiteSparse for Ceres" FORCE)
add_subdirectory(${CERES_DIR} ${CMAKE_BINARY_DIR}/ceres-solver EXCLUDE_FROM_ALL)

# Build Sophus from thirdparty
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Build Sophus tests" FORCE)
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "Build Sophus examples" FORCE)
add_subdirectory(${THIRDPARTY_DIR}/Sophus ${CMAKE_BINARY_DIR}/Sophus)

# Build Pangolin from thirdparty
set(BUILD_PANGOLIN_EXAMPLES OFF CACHE BOOL "Build Pangolin examples" FORCE)
set(BUILD_PANGOLIN_TESTS OFF CACHE BOOL "Build Pangolin tests" FORCE)
add_subdirectory(${THIRDPARTY_DIR}/pangolin ${CMAKE_BINARY_DIR}/pangolin EXCLUDE_FROM_ALL)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${LIGHTWEIGHT_VIO_DIR}/src
  ${THIRDPARTY_DIR}/Sophus
)

# Collect lightweight_vio source files
set(LIGHTWEIGHT_VIO_SOURCES
    ${LIGHTWEIGHT_VIO_DIR}/src/database/Frame.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/database/Feature.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/database/MapPoint.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/processing/FeatureTracker.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/processing/Optimizer.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/processing/Estimator.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/processing/IMUHandler.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/optimization/Factors.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/optimization/Parameters.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/util/Config.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/util/EurocUtils.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/util/TUMUtils.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/util/StringUtils.cpp
    ${LIGHTWEIGHT_VIO_DIR}/src/viewer/PangolinViewer.cpp
)

# VIO ROS2 Node - Direct linking approach
add_executable(vio_node
  src/vio_node.cpp
  ${LIGHTWEIGHT_VIO_SOURCES}
)

ament_target_dependencies(vio_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs  # ⭐ Added for camera frustum
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(vio_node
  ${OpenCV_LIBRARIES}
  ceres
  yaml-cpp
  ${SPDLOG_LIBRARIES}
  Sophus::Sophus
  pango_display
  pango_windowing
  pango_opengl
  pango_core
  pango_image
)

target_include_directories(vio_node PRIVATE
  ${SPDLOG_INCLUDE_DIRS}
)

if(TARGET Eigen3::Eigen)
    target_link_libraries(vio_node Eigen3::Eigen)
else()
    target_include_directories(vio_node PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

# Install targets
install(TARGETS
  vio_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install config files from lightweight_vio
install(DIRECTORY
  lightweight_vio/config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
