set(DEPENDENCY_LIBS 
  ${catkin_LIBRARIES}
  #${OpenSim_LIBRARIES}
  #${opensimrt_LIBRARIES}
  osrtRosUIMU
)

#add_executable(OnlineLowerLimb src/osrt_ros/UIMU/nodes/OnlineLowerLimbUIMUIK.cpp)
add_executable(OnlineLowerLimb nodes/OnlineLowerLimbUIMUIK.cpp)

add_dependencies(OnlineLowerLimb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(OnlineLowerLimb
  ${DEPENDENCY_LIBS} osrtRosPipeline
)
add_executable(OnlineUpperLimb nodes/OnlineUpperLimbUIMUIK.cpp)

add_dependencies(OnlineUpperLimb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(OnlineUpperLimb
  ${DEPENDENCY_LIBS} osrtRosPipeline
)


#add_executable(contact_force_based_phase_detector src/ContactForceBasedPhaseDetector.cpp src/grfm_ros.cpp)

#add_dependencies(contact_force_based_phase_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(contact_force_based_phase_detector
#  ${catkin_LIBRARIES}
#  ${OpenSim_LIBRARIES}
#  ${opensimrt_LIBRARIES}
#)


#TODO: add UIMU as library
