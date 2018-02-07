pkg_search_module(base-types REQUIRED base-types)
include_directories(${base-types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_executable(test_wbc-solvers test_wbc-solvers.cpp ../suite.cpp)
target_link_libraries(test_wbc-solvers 
                      wbc-solvers
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
