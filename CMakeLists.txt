cmake_minimum_required(VERSION 3.0.2)
project(osrt_ros)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

add_compile_options(-std=c++17 -O3 -g)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -g")

find_package(OpenSim 4.0 REQUIRED PATHS "${OPENSIM_HOME}"
	#	Simbody
	)

find_package(catkin REQUIRED COMPONENTS
	message_runtime
	opensimrt
	opensimrt_msgs
	roscpp
	sensor_msgs
	tf
	tf2
	)
#find_package(Simbody REQUIRED PATHS "/opt/dependencies/opensim-core/dependencies/simbody/lib/cmake/simbody/")

find_package(opensimrt REQUIRED COMPONENTS
	Common
	Realtime)


catkin_package(CATKIN_DEPENDS 
	message_runtime
	opensimrt_msgs 
	roscpp
	sensor_msgs
	tf2
	)

message(NOTICE "OpenSim")
message(STATUS
	${OpenSim_INCLUDE_DIRS}

	)
message(NOTICE "Simbody")
message(STATUS
	${Simbody_INCLUDE_DIRS}

	)

message(NOTICE "catkin")
foreach(dir ${catkin_INCLUDE_DIRS})
	message(STATUS
		"catkin_dir=${dir}"
		)
endforeach()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)

file (GLOB Pipeline_SOURCES CONFIGURE_DEPENDS "src/osrt_ros/Pipeline/*.cpp")
file (GLOB Pipeline_HEADERS CONFIGURE_DEPENDS "include/Pipeline/*.h")

set(target osrtRosPipeline)

include_directories("/opt/dependencies/opensim-core/include/OpenSim/"
	"/opt/dependencies/opensim-core/include/simbody/"
	)

add_library(osrtRosPipeline ${Pipeline_HEADERS} ${Pipeline_SOURCES} )

target_link_libraries(osrtRosPipeline ${catkin_LIBRARIES} ${OpenSim_LIBRARIES})

add_dependencies(osrtRosPipeline ${catkin_EXPORTED_TARGETS})

add_subdirectory(src/osrt_ros/Pipeline)



file (GLOB UIMU_SOURCES CONFIGURE_DEPENDS "src/osrt_ros/UIMU/*.cpp")
file (GLOB UIMU_HEADERS CONFIGURE_DEPENDS "include/UIMU/*.h")

set(target osrtRosUIMU)

add_library(osrtRosUIMU ${UIMU_HEADERS} ${UIMU_SOURCES} )

add_dependencies(osrtRosUIMU ${catkin_EXPORTED_TARGETS})

target_link_libraries(osrtRosUIMU ${catkin_LIBRARIES} ${OpenSim_LIBRARIES} ${opensimrt_LIBRARIES})

add_subdirectory(src/osrt_ros/UIMU)

add_executable(custom_joints_node src/osrt_ros/custom_joints_node.cpp)

add_dependencies(custom_joints_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(custom_joints_node
  ${catkin_LIBRARIES}
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
  osrtRosPipeline
)

#add_executable(contact_force_based_phase_detector src/ContactForceBasedPhaseDetector.cpp src/grfm_ros.cpp)

#add_dependencies(contact_force_based_phase_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(contact_force_based_phase_detector
#  ${catkin_LIBRARIES}
#  ${OpenSim_LIBRARIES}
#  ${opensimrt_LIBRARIES}
#)

