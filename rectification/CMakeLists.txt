# Source
set(SRC 
	Undistorter.cpp
	UndistorterKannalaBrandt.cpp
)
set(HDR
	rectification/Undistorter.h
	rectification/UndistorterKannalaBrandt.h
)

# Target
add_library(rectification ${SRC} ${HDR})
target_include_directories(rectification PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(rectification PUBLIC
	${OpenCV_LIBS}
)

# Installations
install(TARGETS rectification
	 EXPORT camera_calibration-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
install(FILES ${HDR} DESTINATION include/rectification)
