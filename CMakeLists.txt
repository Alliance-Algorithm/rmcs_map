cmake_minimum_required(VERSION 3.10)
project(rmcs_map)

set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++20)

set(PACKAGES
    # ros2 origin packages
    rosidl_default_generators
    rclcpp
    geometry_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    # ros2 custom packages
    livox_ros_driver2
    # third party packages
    Eigen
    OpenCV
    # end of packages set
)

aux_source_directory(src/ros2 PROJECT_SOURCE)
aux_source_directory(src/map PROJECT_SOURCE)

# rmeove pcl warning
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find package
foreach(package ${PACKAGES})
  find_package(${package} REQUIRED)
  if(${package}_FOUND)
    message("found package: ${package}")
  endif()
endforeach()

include_directories(${PACKAGES}_INCLUDE_DIR src)

# generate custom message
rosidl_generate_interfaces(
  ${PROJECT_NAME} msg/RobotStatus.msg msg/GameStatus.msg DEPENDENCIES
  builtin_interfaces geometry_msgs)
rosidl_get_typesupport_target(CUSTOM_MESSAGE ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")

# executable for offline
add_executable(${PROJECT_NAME}_exe ${PROJECT_SOURCE} src/offline.cpp)
ament_target_dependencies(${PROJECT_NAME}_exe ${PACKAGES})
target_link_libraries(${PROJECT_NAME}_exe ${CUSTOM_MESSAGE})
install(TARGETS ${PROJECT_NAME}_exe DESTINATION lib/${PROJECT_NAME})

# install directory
install(DIRECTORY config msg launch DESTINATION share/${PROJECT_NAME})

# generate ament index for other package
ament_package()
