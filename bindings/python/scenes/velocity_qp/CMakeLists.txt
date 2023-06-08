PYTHON_ADD_MODULE(velocity_scene_qp velocity_scene_qp.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(velocity_scene_qp PUBLIC
	      wbc-scenes-velocity_qp
        wbc-robot_models-rbdl
        wbc-solvers-qpoases
        Boost::python
        Boost::numpy)

target_include_directories(velocity_scene_qp PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS velocity_scene_qp
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/scenes)
