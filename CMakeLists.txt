cmake_minimum_required(VERSION 3.10)
project(laser_odom_calibration)

SET(CSM_SRC 
  src/csm/laser_data.cpp
  src/csm/math_utils.cpp
  src/csm/math_utils_gsl.cpp
  src/csm/utils.cpp
  src/csm/logging.cpp
  src/csm/sm_options.cpp
  src/csm/orientation.cpp
  src/csm/clustering.cpp)

SET(EGSL_SRC
	src/egsl/egsl.cpp
	src/egsl/egsl_conversions.cpp
	src/egsl/egsl_misc.cpp
	src/egsl/egsl_ops.cpp)

SET(GPC_SRC
	src/gpc/gpc.cpp
	src/gpc/gpc_utils.cpp)

SET(ICP_SRC
  src/icp/icp.cpp
  src/icp/icp_corr_dumb.cpp
  src/icp/icp_corr_tricks.cpp
  src/icp/icp_covariance.cpp
  src/icp/icp_debug.cpp
  src/icp/icp_loop.cpp
  src/icp/icp_outliers.cpp)

SET(OPTIONS_SRC  
   src/options/options.cpp
   src/options/options_interface.cpp)

ADD_LIBRARY(csm_eigen
	${CSM_SRC}
	${EGSL_SRC} 
	${GPC_SRC}
	${ICP_SRC}
	${OPTIONS_SRC})

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPENDENCIES
  Eigen3
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  rosbag2_cpp
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(csm_eigen Eigen3)

target_include_directories(csm_eigen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# A test
# ADD_EXECUTABLE(test_calib src/main.cpp)
# TARGET_LINK_LIBRARIES(test_calib ${CSM_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE(io_test src/io_test.cpp src/io.cpp src/utils.cpp src/scan_match.cpp src/synchronizer.cpp src/solver.cpp)
target_link_libraries(io_test
  csm_eigen
)
ament_target_dependencies(io_test
  ${DEPENDENCIES}
)
install(
  TARGETS
    csm_eigen
    io_test
  # EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}       # <-- executables
  LIBRARY DESTINATION lib/${PROJECT_NAME}       # <-- shared-objects
  ARCHIVE DESTINATION lib/${PROJECT_NAME}       # <-- static-libs
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${DEPENDENCIES}
)
ament_package()