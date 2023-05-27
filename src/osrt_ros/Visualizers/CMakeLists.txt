message(${opensimrt_LIBRARIES})


set(DEPENDENCY_LIBS 
  ${catkin_LIBRARIES}
  osrtRosVisualizers
)

set(CMAKE_VERBOSE_MAKEFILE ON)

foreach(lib ${DEPENDENCY_LIBS})
  message(STATUS "lib=${lib}")
endforeach()


get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_executable(vis_grf nodes/grf_vis.cpp)

add_dependencies(vis_grf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vis_grf
	${DEPENDENCY_LIBS}
	)

add_executable(vis_so nodes/so_vis.cpp)

add_dependencies(vis_so ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vis_so
	${DEPENDENCY_LIBS}
)
add_executable(vis_id nodes/id_vis.cpp)

add_dependencies(vis_id ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vis_id
	${DEPENDENCY_LIBS}
)

add_executable(vis_ik nodes/ik_vis.cpp)

add_dependencies(vis_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vis_ik
	${DEPENDENCY_LIBS}
)
set(CMAKE_VERBOSE_MAKEFILE OFF)
