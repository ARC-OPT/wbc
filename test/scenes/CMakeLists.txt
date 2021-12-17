find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(orocos-kdl REQUIRED orocos-kdl)
pkg_search_module(kdl_parser REQUIRED kdl_parser)
pkg_search_module(qpOASES REQUIRED qpOASES)

include_directories(${base-types_INCLUDE_DIRS}
                    ${orocos-kdl_INCLUDE_DIRS}
                    ${kdl_parser_INCLUDE_DIRS}
                    ${qpOASES_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${orocos-kdl_LIBRARY_DIRS}
                 ${kdl_parser_LIBRARY_DIRS}
                 ${qpOASES_LIBRARY_DIRS})

add_executable(test_velocity_scene test_velocity_scene.cpp ../suite.cpp)
target_link_libraries(test_velocity_scene
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-tools
                      wbc-solvers-hls
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_velocity_scene_quadratic_cost test_velocity_scene_quadratic_cost.cpp ../suite.cpp)
target_link_libraries(test_velocity_scene_quadratic_cost
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-solvers-qpoases
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_acceleration_scene test_acceleration_scene.cpp ../suite.cpp)
target_link_libraries(test_acceleration_scene
                      wbc-scenes
                      wbc-robot_models-kdl
                      wbc-tools
                      wbc-solvers-qpoases
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})


