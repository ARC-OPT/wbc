pkg_search_module(base-types REQUIRED base-types)
include_directories(${base-types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/src)
add_executable(cart_pos_ctrl_hls cart_pos_ctrl_hls.cpp)
target_link_libraries(cart_pos_ctrl_hls
                      wbc-solvers-hls
                      wbc-scenes
                      wbc-robot_models)
