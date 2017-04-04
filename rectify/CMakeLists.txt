# Source
set(SRC rectify.cpp)

# Target
add_executable(rectify ${SRC})
target_include_directories(rectify PRIVATE
	${Boost_INCLUDE_DIRS}
	${VSAL_INCLUDE_DIRS}
)
target_link_libraries(rectify PRIVATE
	rectification
	${Boost_LIBRARIES}
	${VSAL_LIBRARIES}
)

# Installations
install(TARGETS rectify EXPORT nirkin_face-targets DESTINATION bin COMPONENT app)