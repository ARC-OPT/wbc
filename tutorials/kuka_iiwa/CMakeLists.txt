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
add_executable(cart_pos_ctrl_hls cart_pos_ctrl_hls.cpp)
target_link_libraries(cart_pos_ctrl_hls
                      wbc-solvers-hls
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-controllers)

add_executable(cart_pos_ctrl_qpoases cart_pos_ctrl_qpoases.cpp)
target_link_libraries(cart_pos_ctrl_qpoases
                      wbc-solvers-qpoases
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-controllers)

if(USE_EIQUADPROG)
add_executable(cart_pos_ctrl_eiquadprog cart_pos_ctrl_eiquadprog.cpp)
target_link_libraries(cart_pos_ctrl_eiquadprog
                      wbc-solvers-eiquadprog
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-controllers)
endif()

add_executable(cart_pos_ctrl_hls_weights cart_pos_ctrl_hls_weights.cpp)
target_link_libraries(cart_pos_ctrl_hls_weights
                      wbc-solvers-hls
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-controllers)

add_executable(cart_pos_ctrl_hls_hierarchies cart_pos_ctrl_hls_hierarchies.cpp)
target_link_libraries(cart_pos_ctrl_hls_hierarchies
                      wbc-solvers-hls
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-controllers)
