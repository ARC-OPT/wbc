PYTHON_ADD_MODULE(acceleration_scene acceleration_scene.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(acceleration_scene PUBLIC
	      wbc-scenes-acceleration
        wbc-robot_models-rbdl
	      wbc-solvers-qpoases
        Boost::python
        Boost::numpy)

target_include_directories(acceleration_scene PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS acceleration_scene
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/scenes)
