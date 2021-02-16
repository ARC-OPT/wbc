pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(orocos-kdl REQUIRED orocos-kdl)
pkg_search_module(kdl_parser REQUIRED kdl_parser)
pkg_search_module(hyrodyn REQUIRED hyrodyn)
include_directories(${base-types_INCLUDE_DIRS}
                    ${orocos-kdl_INCLUDE_DIRS}
                    ${kdl_parser_INCLUDE_DIRS}
                    ${hyrodyn_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${orocos-kdl_LIBRARY_DIRS}
                 ${kdl_parser_LIBRARY_DIRS}
                 ${hyrodyn_LIBRARY_DIRS})

add_executable(upper_body_exo_with_rh5 upper_body_exo_with_rh5.cpp)
target_link_libraries(upper_body_exo_with_rh5
                      wbc-robot_models)

