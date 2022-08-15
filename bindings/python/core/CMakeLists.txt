PYTHON_ADD_MODULE(core core.cpp)
# Set up the libraries and header search paths for this target
target_link_libraries(core PUBLIC
	${PYTHON_LIBRARIES}
	Boost::python3
	Boost::numpy3
	wbc-core)

target_include_directories(core PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS core
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
