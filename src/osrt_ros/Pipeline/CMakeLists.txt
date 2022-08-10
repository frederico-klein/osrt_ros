message(${opensimrt_LIBRARIES})


set(DEPENDENCY_LIBS 
  ${catkin_LIBRARIES}
  #${OpenSim_LIBRARIES}
  #${opensimrt_LIBRARIES}
  osrtRosPipeline
)

set(CMAKE_VERBOSE_MAKEFILE ON)

foreach(lib ${DEPENDENCY_LIBS})
  message(STATUS "lib=${lib}")
endforeach()


get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_executable(cgrf nodes/cgrf.cpp)

add_dependencies(cgrf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cgrf
	${DEPENDENCY_LIBS}
	)

add_executable(agrf nodes/agrf.cpp)

add_dependencies(agrf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(agrf
	${DEPENDENCY_LIBS}
)
add_executable(id_so_jr nodes/id_combined.cpp)

add_dependencies(id_so_jr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(id_so_jr
	${DEPENDENCY_LIBS}
)

add_executable(id nodes/id_pipeline.cpp)

add_dependencies(id ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(id
	${DEPENDENCY_LIBS}
)
set(CMAKE_VERBOSE_MAKEFILE OFF)
