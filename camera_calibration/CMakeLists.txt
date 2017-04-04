# Source
set(SRC 
	CameraCalibration.cpp
	Chessboard.cpp
)
set(HDR 
	camera_calibration/CameraCalibration.h
	camera_calibration/Chessboard.h
	camera_calibration/ChessboardCorner.h
	camera_calibration/ChessboardQuad.h
	camera_calibration/Spline.h
)

# Target
add_library(camera_calibration ${SRC} ${HDR})
target_include_directories(camera_calibration PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${VSAL_INCLUDE_DIRS}
)
target_link_libraries(camera_calibration PUBLIC
	camera_models
	cauldron
	sparse_graph
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	${VSAL_LIBRARIES}
)

# Installations
install(TARGETS camera_calibration
	 EXPORT camera_calibration-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
install(FILES ${HDR} DESTINATION include/camera_calibration)
