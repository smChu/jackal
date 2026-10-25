cmake_minimum_required(VERSION 3.16)
project(jackal_actor_controller)

# Let editors use the same include paths and generated definitions as the build.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-rendering6 REQUIRED)

add_library(${PROJECT_NAME} SHARED src/ActorController.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs)
target_link_libraries(${PROJECT_NAME}
  ignition-gazebo6::ignition-gazebo6 ignition-plugin1::register ignition-rendering6::ignition-rendering6)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
ament_environment_hooks(env-hooks/actor_controller.dsv.in)

ament_package()
