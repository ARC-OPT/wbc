PYTHON_ADD_MODULE(acceleration_scene_tsid acceleration_scene_tsid.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(acceleration_scene_tsid PUBLIC
	      wbc-scenes-acceleration_tsid
        wbc-robot_models-rbdl
	      wbc-solvers-qpoases
        Boost::python
        Boost::numpy)

target_include_directories(acceleration_scene_tsid PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS acceleration_scene_tsid
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/scenes)
