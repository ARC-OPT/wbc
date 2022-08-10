PYTHON_ADD_MODULE(robot_model_kdl robot_model_kdl.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(robot_model_kdl PUBLIC
	${PYTHON_LIBRARIES}
	Boost::python3
	Boost::numpy3
	wbc-robot_models-kdl)

target_include_directories(robot_model_kdl PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS robot_model_kdl
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/robot_models)
