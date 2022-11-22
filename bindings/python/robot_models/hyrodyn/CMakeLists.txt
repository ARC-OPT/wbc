PYTHON_ADD_MODULE(robot_model_hyrodyn robot_model_hyrodyn.cpp)
target_link_libraries(robot_model_hyrodyn PUBLIC
	${PYTHON_LIBRARIES}
	Boost::python
	Boost::numpy
	wbc-robot_models-hyrodyn)

target_include_directories(robot_model_hyrodyn PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS robot_model_hyrodyn
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/robot_models)
