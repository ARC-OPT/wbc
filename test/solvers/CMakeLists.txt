find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(qpOASES REQUIRED qpOASES)
include_directories(${base-types_INCLUDE_DIRS}
                    ${qpOASES_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${qpOASES_LIBRARY_DIRS})


add_executable(hls_solver hls_solver.cpp ../suite.cpp)
target_link_libraries(hls_solver
                      wbc-solvers-hls
                      wbc-types
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})


add_executable(qp_oases_solver qp_oases_solver.cpp ../suite.cpp)
target_link_libraries(qp_oases_solver
                      wbc-solvers-qpoases
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
