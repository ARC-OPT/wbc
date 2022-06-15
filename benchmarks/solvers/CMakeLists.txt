find_package(Boost COMPONENTS system filesystem REQUIRED)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(orocos-kdl REQUIRED orocos-kdl)
pkg_search_module(qpOASES REQUIRED qpOASES)
pkg_search_module(kdl_parser REQUIRED kdl_parser)
include_directories(${base-types_INCLUDE_DIRS}
                    ${orocos-kdl_INCLUDE_DIRS}
                    ${qpOASES_INCLUDE_DIRS}
                    ${kdl_parser_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${orocos-kdl_LIBRARY_DIRS}
                 ${qpOASES_LIBRARY_DIRS}
                 ${kdl_parser_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/src)
add_executable(benchmark_solvers benchmark_solvers.cpp ../benchmarks_common.cpp)
target_link_libraries(benchmark_solvers
                      wbc-solvers-hls
                      wbc-solvers-qpoases
                      wbc-solvers-qpswift
                      wbc-scenes
                      wbc-robot_models-hyrodyn
                      wbc-controllers
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY})

