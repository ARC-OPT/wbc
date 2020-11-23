pkg_search_module(hyrodyn REQUIRED hyrodyn)
pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(kdl_parser REQUIRED kdl_parser)
pkg_search_module(rbdl REQUIRED rbdl)

include_directories(${hyrodyn_INCLUDE_DIRS}
                    ${base-types_INCLUDE_DIRS}
                    ${kdl_parser_INCLUDE_DIRS}
                    ${rbdl_INCLUDE_DIRS})
link_directories(${hyrodyn_LIBRARY_DIRS}
                 ${base-types_INCLUDE_DIRS}
                 ${kdl_parser_LIBRARY_DIRS}
                 ${rbdl_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/src)
add_executable(rh5_one_leg_floating_base rh5_one_leg_floating_base.cpp)
target_link_libraries(rh5_one_leg_floating_base
                      wbc-solvers-qpoases
                      wbc-scenes
                      wbc-tools
                      wbc-robot_models
                      ${hyrodyn_LIBRARIES}
                      ${rbdl_LIBRARIES})
add_executable(rh5_one_leg rh5_one_leg.cpp)
target_link_libraries(rh5_one_leg
                      wbc-solvers-qpoases
                      wbc-scenes
                      wbc-tools
                      wbc-robot_models)

