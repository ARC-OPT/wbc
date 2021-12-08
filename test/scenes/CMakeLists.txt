find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(orocos-kdl REQUIRED orocos-kdl)
pkg_search_module(kdl_parser REQUIRED kdl_parser)

include_directories(${base-types_INCLUDE_DIRS}
                    ${orocos-kdl_INCLUDE_DIRS}
                    ${kdl_parser_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${orocos-kdl_LIBRARY_DIRS}
                 ${kdl_parser_LIBRARY_DIRS})

add_executable(wbc_velocity_scene wbc_velocity_scene.cpp ../suite.cpp)
target_link_libraries(wbc_velocity_scene
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-solvers-hls
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
