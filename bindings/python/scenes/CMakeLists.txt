PYTHON_ADD_MODULE(scenes scenes.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(scenes PUBLIC
	wbc-scenes
	wbc-robot_models-kdl
	wbc-solvers-hls
        wbc-solvers-qpoases
        Boost::python3
        Boost::numpy3)

target_include_directories(scenes PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS scenes
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
