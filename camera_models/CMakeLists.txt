# Source
set(SRC 
	Camera.cpp
	CameraFactory.cpp
	CataCamera.cpp
	CostFunctionFactory.cpp
	EquidistantCamera.cpp
	PinholeCamera.cpp
)
set(HDR 
	camera_models/Camera.h
	camera_models/CameraFactory.h
	camera_models/CataCamera.h
	camera_models/CostFunctionFactory.h
	camera_models/EquidistantCamera.h
	camera_models/PinholeCamera.h
)

# Target
add_library(camera_models ${SRC} ${HDR})
target_include_directories(camera_models PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_link_libraries(camera_models PUBLIC
	cauldron
)

# Installations
install(TARGETS camera_models
	 EXPORT camera_calibration-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
install(FILES ${HDR} DESTINATION include/camera_models)
