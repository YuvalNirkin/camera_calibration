# Source
set(SRC mono_camera_calibration.cpp)

# Target
add_executable(mono_camera_calibration ${SRC})
target_include_directories(mono_camera_calibration PRIVATE
	${Boost_INCLUDE_DIRS}
	${VSAL_INCLUDE_DIRS}
)
target_link_libraries(mono_camera_calibration PRIVATE
	camera_calibration
	${Boost_LIBRARIES}
	${VSAL_LIBRARIES}
)

# Installations
install(TARGETS mono_camera_calibration EXPORT nirkin_face-targets DESTINATION bin COMPONENT app)
install(FILES mono_camera_calibration.cfg DESTINATION bin COMPONENT app)