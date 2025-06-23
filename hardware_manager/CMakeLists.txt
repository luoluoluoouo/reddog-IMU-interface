cmake_minimum_required(VERSION 3.8)
project(hardware_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

find_package(xsens_mti_ros2_driver REQUIRED)

link_directories(lib/xspublic/xscontroller lib/xspublic/xscommon lib/xspublic/xstypes)

add_executable(main_exec 
  src/main.cpp
  ../xsens_mti_ros2_driver/src/xdainterface.cpp
  ../xsens_mti_ros2_driver/src/xdacallback.cpp
  ../xsens_mti_ros2_driver/src/xsens_log_handler.cpp
  ../xsens_mti_ros2_driver/src/ntrip_util.cpp
  ../xsens_mti_ros2_driver/src/xsens_time_handler.cpp 
)
target_link_libraries(main_exec
  xscontroller
  xscommon
  xstypes
)
ament_target_dependencies(main_exec 
  rclcpp
  mavros_msgs
  tf2_ros
  xsens_mti_ros2_driver
  nmea_msgs
  nav_msgs
)
target_include_directories(main_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
  ${CMAKE_CURRENT_SOURCE_DIR}/../xsens_mti_ros2_driver/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../xsens_mti_ros2_driver/src/messagepublishers
  ${CMAKE_CURRENT_SOURCE_DIR}/../xsens_mti_ros2_driver/msg
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/../xsens_mti_ros2_driver/rosidl_generator_cpp>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/nmea_msgs/rosidl_generator_cpp> 
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/nav_msgs/rosidl_generator_cpp>
)

add_executable(stand_go2 src/dds.cpp)
target_link_libraries(stand_go2  
  unitree_sdk2
)
ament_target_dependencies(stand_go2 
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS main_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS stand_go2
  DESTINATION lib/${PROJECT_NAME}
)

SET(CMAKE_BUILD_TYPE Release)
ament_package()