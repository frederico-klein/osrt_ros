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

add_executable(grf_vis nodes/grf_vis.cpp)

add_dependencies(grf_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(grf_vis
	${DEPENDENCY_LIBS}
	)

add_executable(so_vis nodes/so_vis.cpp)

add_dependencies(so_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(so_vis
	${DEPENDENCY_LIBS}
)
add_executable(id_vis nodes/id_vis.cpp)

add_dependencies(id_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(id_vis
	${DEPENDENCY_LIBS}
)

add_executable(ik_vis nodes/ik_vis.cpp)

add_dependencies(ik_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ik_vis
	${DEPENDENCY_LIBS}
)
set(CMAKE_VERBOSE_MAKEFILE OFF)
