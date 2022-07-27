find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(test_eiquadprog_solver test_eiquadprog_solver.cpp ../../suite.cpp)
target_link_libraries(test_eiquadprog_solver
                      wbc-solvers-eiquadprog
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
