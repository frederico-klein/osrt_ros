set(DEPENDENCY_LIBS 
  ${catkin_LIBRARIES}
  #${OpenSim_LIBRARIES}
  #${opensimrt_LIBRARIES}
  osrtRosUIMU
)

add_executable(OnlineLowerLimb nodes/OnlineLowerLimbUIMUIK.cpp)

add_dependencies(OnlineLowerLimb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(OnlineLowerLimb
  ${DEPENDENCY_LIBS} osrtRosPipeline
)
add_executable(OnlineUpperLimb nodes/OnlineUpperLimbUIMUIK.cpp)

add_dependencies(OnlineUpperLimb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(OnlineUpperLimb
  ${DEPENDENCY_LIBS} osrtRosPipeline
)


