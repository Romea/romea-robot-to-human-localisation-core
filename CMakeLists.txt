cmake_minimum_required(VERSION 3.5)
project(romea_robot_to_human_localisation_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_localisation REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_localisation_msgs REQUIRED)
find_package(romea_localisation_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/robot_to_human_localisation_filter.cpp
  src/robot_to_human_localisation.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "ROMEA_ROBOT_TO_HUMAN_LOCALISATION_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME}
  class_loader
  rclcpp
  rclcpp_components
  romea_core_common
  romea_core_localisation
  romea_common_utils
  romea_localisation_msgs
  romea_localisation_utils)

if(${ROS_DISTRO} STREQUAL "ROS_GALACTIC")
  rclcpp_components_register_nodes(${PROJECT_NAME} "romea::R2WKalmanLocalisation")

  add_executable(robot_to_human_localisation_node
    src/robot_to_human_localisation_filter.cpp
    src/robot_to_human_localisation.cpp
    src/robot_to_human_localisation_node.cpp)

  target_include_directories(robot_to_human_localisation_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  ament_target_dependencies(robot_to_human_localisation_node
    class_loader
    rclcpp
    rclcpp_components
    romea_core_common
    romea_core_localisation
    romea_common_utils
    romea_localisation_msgs
    romea_localisation_utils)

  install(TARGETS
    robot_to_human_localisation_node
    DESTINATION lib/${PROJECT_NAME})
else()
  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "romea::R2HLocalisation"
    EXECUTABLE robot_to_human_localisation_node
    EXECUTOR "MultiThreadedExecutor"
  )
endif()


install(TARGETS
 ${PROJECT_NAME}
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin)


#add_executable(${PROJECT_NAME}_node2 src/robot_to_world_localisation_node.cpp)
#target_link_libraries(${PROJECT_NAME}_node2 ${PROJECT_NAME})
#ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components pluginlib)

#install(TARGETS
# ${PROJECT_NAME}_node2
# DESTINATION lib/${PROJECT_NAME})

#ament_export_dependencies(eigen3_cmake_module)
#ament_export_dependencies(Eigen3)
#ament_export_dependencies(geometry_msgs)
#ament_export_dependencies(romea_corecommon)
#ament_export_dependencies(romea_common_utils)
#ament_export_dependencies(four_wheel_steering_msgs)
#ament_export_dependencies(ackermann_msgs)
#ament_export_dependencies(romea_mobile_base)
#ament_export_dependencies(romea_mobile_base_msgs)
#ament_export_dependencies(romea_mobile_base_utils)
#ament_export_dependencies(romea_localisation_msgs)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#install(
#  DIRECTORY include/
#  DESTINATION include
#)

#install(
#  TARGETS ${PROJECT_NAME}
#  EXPORT export_${PROJECT_NAME}
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#  RUNTIME DESTINATION bin
#  INCLUDES DESTINATION include
#)

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  #  add_subdirectory(test)

endif()

ament_package()


