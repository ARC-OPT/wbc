find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(eiquadprog REQUIRED eiquadprog)
include_directories(${base-types_INCLUDE_DIRS} ${eiquadprog_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${eiquadprog_LIBRARY_DIRS})

add_executable(test_eiquadprog_solver test_eiquadprog_solver.cpp ../../suite.cpp)
target_link_libraries(test_eiquadprog_solver
                      wbc-solvers-eiquadprog
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
