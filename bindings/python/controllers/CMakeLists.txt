PYTHON_ADD_MODULE(controllers controllers.cpp)

# Set up the libraries and header search paths for this target
target_link_libraries(controllers PUBLIC
	${PYTHON_LIBRARIES}
	Boost::python3
	Boost::numpy3
	wbc-controllers)

target_include_directories(controllers PUBLIC
	${PYTHON_INCLUDE_DIRS})

install(TARGETS controllers
        DESTINATION ${PYTHON_INSTALL_PATH}/wbc)
