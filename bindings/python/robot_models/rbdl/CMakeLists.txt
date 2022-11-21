PYTHON_ADD_MODULE(robot_model_rbdl robot_model_rbdl.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(robot_model_rbdl PUBLIC
	${PYTHON_LIBRARIES}
	Boost::python
	Boost::numpy
	wbc-robot_models-rbdl)

target_include_directories(robot_model_rbdl PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS robot_model_rbdl
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/robot_models)
