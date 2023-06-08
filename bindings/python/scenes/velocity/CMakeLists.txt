PYTHON_ADD_MODULE(velocity_scene velocity_scene.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(velocity_scene PUBLIC
	      wbc-scenes-velocity
        wbc-robot_models-rbdl
	      wbc-solvers-hls
        Boost::python
        Boost::numpy)

target_include_directories(velocity_scene PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS velocity_scene
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/scenes)
