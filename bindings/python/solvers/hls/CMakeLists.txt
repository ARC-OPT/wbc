PYTHON_ADD_MODULE(hls_solver HierarchicalLSSolver.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(hls_solver PUBLIC
	${PYTHON_LIBRARIES}
	Boost::python3
	Boost::numpy3
	wbc-solvers-hls)

target_include_directories(hls_solver PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS hls_solver
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc/solvers)
