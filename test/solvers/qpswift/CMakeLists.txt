find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(qpSWIFT REQUIRED qpSWIFT)
include_directories(${base-types_INCLUDE_DIRS}
                    ${qpSWIFT_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${qpSWIFT_LIBRARY_DIRS})

add_executable(test_qpswift_solver test_qpswift_solver.cpp ../../suite.cpp)
target_link_libraries(test_qpswift_solver
                      wbc-solvers-qpswift
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
