cmake_minimum_required(VERSION 3.0.2)
project(opensimrt_ros)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  opensimrt
  opensimrt_msgs
  roscpp
  tf
)

find_package(OpenSim REQUIRED)

catkin_package(
  CATKIN_DEPENDS opensimrt_msgs roscpp
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(acceleration_based_phase_detector src/AccelerationBasedPhaseDetector.cpp)

add_dependencies(acceleration_based_phase_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(acceleration_based_phase_detector
  ${catkin_LIBRARIES}
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
)

add_executable(contact_force_based_phase_detector src/ContactForceBasedPhaseDetector.cpp)

add_dependencies(contact_force_based_phase_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(contact_force_based_phase_detector
  ${catkin_LIBRARIES}
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
)

