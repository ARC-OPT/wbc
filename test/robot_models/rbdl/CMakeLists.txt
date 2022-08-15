find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(rbdl REQUIRED rbdl URDFReader)
pkg_search_module(base-types REQUIRED base-types)

include_directories(${rbdl_INCLUDE_DIRS} ${base-types_INCLUDE_DIRS})
link_directories(${rbdl_LIBRARY_DIRS} ${base-types_LIBRARY_DIRS})

add_executable(test_robot_model_rbdl test_robot_model_rbdl.cpp ../../suite.cpp ../test_robot_model.cpp)
target_link_libraries(test_robot_model_rbdl
                      wbc-robot_models-rbdl
                      wbc-tools
                      wbc-core
                      ${rbdl_LIBRARIES}
                      ${RBDL_URDFReader_LIBRARY}
                      ${base-types_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

