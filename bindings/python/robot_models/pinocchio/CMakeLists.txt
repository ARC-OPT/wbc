PYTHON_ADD_MODULE(robot_model_pinocchio robot_model_pinocchio.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(robot_model_pinocchio PUBLIC
	${PYTHON_LIBRARIES}
	Boost::python
        Boost::numpy
	wbc-robot_models-pinocchio)

target_include_directories(robot_model_pinocchio PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS robot_model_pinocchio
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/robot_models)
