# Source
set(SRC 
	Transform.cpp
)
set(HDR 
	sparse_graph/Transform.h
)

# Target
add_library(sparse_graph ${SRC} ${HDR})
target_include_directories(sparse_graph PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(sparse_graph PUBLIC
	cauldron
)

# Installations
install(TARGETS sparse_graph
	 EXPORT camera_calibration-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
install(FILES ${HDR} DESTINATION include/sparse_graph)
