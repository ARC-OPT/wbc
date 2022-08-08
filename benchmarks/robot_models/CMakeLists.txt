find_package(Boost COMPONENTS system filesystem REQUIRED)

FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(orocos-kdl REQUIRED orocos-kdl)
pkg_search_module(kdl_parser REQUIRED kdl_parser)
pkg_search_module(rbdl REQUIRED rbdl)
include_directories(${base-types_INCLUDE_DIRS}
                    ${orocos-kdl_INCLUDE_DIRS}
                    ${kdl_parser_INCLUDE_DIRS}
                    ${rbdl_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${orocos-kdl_LIBRARY_DIRS}
                 ${kdl_parser_LIBRARY_DIRS}
                 ${rbdl_LIBRARY_DIRS})

include_directories(${base-types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${wbc-robot_models-rbdl_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/src)
add_executable(benchmark_robot_models benchmark_robot_models.cpp ../benchmarks_common.cpp )
target_link_libraries(benchmark_robot_models
                      wbc-core
                      wbc-robot_models-kdl
                      wbc-robot_models-hyrodyn
                      wbc-robot_models-pinocchio
                      wbc-robot_models-rbdl
                      ${rbdl_LIBRARIES}
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY}
                      ${RBDL_URDFReader_LIBRARY})

