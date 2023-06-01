cmake_minimum_required(VERSION 3.8)
project(onsmc_rt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# -- custom dependencies --
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#find_package(nn REQUIRED)
#find_package(onsmc REQUIRED)

# bc we use custom interfaces:
find_package(rosidl_default_generators REQUIRED)
set(msg_files "msg/Time.msg" "msg/DesiredTrajectory.msg" "msg/Control.msg" "msg/State.msg")
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})
ament_export_dependencies(rosidl_default_runtime)


# -- executables --
add_executable(time_publisher src/time_publisher.cpp)
ament_target_dependencies(time_publisher rclcpp std_msgs)

add_executable(trajectory_generator src/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator rclcpp)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp)

add_executable(reader src/reader.cpp)
ament_target_dependencies(reader rclcpp)

add_executable(commander src/commander.cpp)
ament_target_dependencies(commander rclcpp)

add_executable(writer src/writer.cpp)
ament_target_dependencies(writer rclcpp std_msgs)

# ----- include stuff ----

add_library(nn src/nn.cpp)
add_library(onsmc  src/onsmc.cpp)
#ament_target_dependencies(onsmc nn)

# to find our header files
# target_include_directories(nn
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

target_include_directories(nn
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(onsmc
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# -- for each executable added:
install(TARGETS
        time_publisher trajectory_generator controller reader commander writer
        DESTINATION lib/${PROJECT_NAME})

# to use launch file:
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

# to use our custom includes:
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME})

# -- also because we use custom interfaces: 
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(time_publisher "${cpp_typesupport_target}")
target_link_libraries(trajectory_generator "${cpp_typesupport_target}")
target_link_libraries(controller "${cpp_typesupport_target}")
target_link_libraries(reader "${cpp_typesupport_target}")
target_link_libraries(commander "${cpp_typesupport_target}")
target_link_libraries(writer "${cpp_typesupport_target}")

target_link_libraries(controller onsmc)
target_link_libraries(onsmc nn)

#target_link_libraries(nn "${cpp_typesupport_target}")
#target_link_libraries(onsmc "${cpp_typesupport_target}")

ament_package()

