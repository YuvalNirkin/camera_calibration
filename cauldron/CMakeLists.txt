# Source
set(SRC 
	cauldron.cpp
	EigenQuaternionParameterization.cpp
	PLine.cpp
	PLineCorrespondence.cpp
)
set(HDR
	cauldron/AtomicContainer.h
	cauldron/cauldron.h
	cauldron/DataBuffer.h
	cauldron/EigenQuaternionParameterization.h
	cauldron/EigenUtils.h
	cauldron/Enums.h
	cauldron/PLine.h
	cauldron/PLineCorrespondence.h
)

# Target
add_library(cauldron ${SRC} ${HDR})
target_include_directories(cauldron PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	${OpenCV_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(cauldron PUBLIC
	${OpenCV_LIBS}
	${CERES_LIBRARIES}
)

# Installations
install(TARGETS cauldron
	 EXPORT camera_calibration-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
install(FILES ${HDR} DESTINATION include/cauldron)
