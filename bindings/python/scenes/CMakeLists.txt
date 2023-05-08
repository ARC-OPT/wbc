PYTHON_ADD_MODULE(scenes scenes.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(scenes PUBLIC
	wbc-scenes
        wbc-robot_models-rbdl
	wbc-solvers-hls
        wbc-solvers-qpoases
        Boost::python
        Boost::numpy)

target_include_directories(scenes PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS scenes
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
