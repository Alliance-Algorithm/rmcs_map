cmake_minimum_required(VERSION 3.10)
project(rmcs_map)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

set(PACKAGES
    rosidl_default_generators
    rclcpp
    geometry_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    livox_ros_driver2
    rmcs_executor
    Eigen
    OpenCV
)

# rmeove pcl warning
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# FIND PACKAGES
foreach(package ${PACKAGES})
  find_package(${package} REQUIRED)
  if(${package}_FOUND)
    message("found package: ${package}")
  endif()
endforeach()

aux_source_directory(src/ros2 SOURCE)
aux_source_directory(src/map SOURCE)

# GENERATE CUSTOM MESSAGE 
rosidl_generate_interfaces(
  ${PROJECT_NAME} msg/RobotStatus.msg msg/GameStatus.msg 
  DEPENDENCIES builtin_interfaces geometry_msgs
)

rosidl_get_typesupport_target(
  CUSTOM_MESSAGE 
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# BUILD RMCS COMPONENT
find_package (ament_cmake_auto REQUIRED)
find_package (pluginlib REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library (
  ${PROJECT_NAME}_dll 
  SHARED 
  src/component.cpp
  ${SOURCE}
)

ament_target_dependencies(
  ${PROJECT_NAME}_dll
  ${PACKAGES}
)

target_link_libraries(
  ${PROJECT_NAME}_dll 
  ${CUSTOM_MESSAGE}
)

# BUILD OFFLINE EXE
add_executable(
  ${PROJECT_NAME}_exe 
  ${SOURCE} 
  src/offline.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_exe 
  ${PACKAGES}
)

target_link_libraries(
  ${PROJECT_NAME}_exe 
  ${CUSTOM_MESSAGE}
)

# INSTALL BUILD FILE
install(
  TARGETS ${PROJECT_NAME}_exe ${PROJECT_NAME}_dll
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config msg launch 
  DESTINATION share/${PROJECT_NAME}
)

include_directories(${PACKAGES}_INCLUDE_DIR)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

ament_package()
