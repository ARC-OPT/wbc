find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories()

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(kdl REQUIRED orocos-kdl)
pkg_search_module(kdl_conversions REQUIRED kdl_conversions-kdl)
pkg_search_module(kdl_parser REQUIRED kdl_parser)
pkg_search_module(wbc_solvers-hls REQUIRED wbc_solvers-hls)

include_directories(${kdl_INCLUDE_DIRS}
                    ${base-types_INCLUDE_DIRS}
                    ${kdl_conversions_INCLUDE_DIRS}
                    ${kdl_parser_INCLUDE_DIRS}
                    ${wbc_solvers-hls_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/src)
link_directories(${kdl_LIBRARY_DIRS}
                 ${kdl_conversions_LIBRARY_DIRS}
                 ${kdl_parser_LIBRARY_DIRS}
                 ${wbc_solvers-hls_LIBRARY_DIRS}
                 ${base-types_LIBRARY_DIRS})

add_executable(test_wbc_velocity_scene test_wbc_velocity_scene.cpp suite.cpp)
add_executable(test_kinematic_robot_model_kdl test_kinematic_robot_model_kdl.cpp suite.cpp)
target_link_libraries(test_wbc_velocity_scene
                      wbc
                      ${kdl_LIBRARIES}
                      ${kdl_conversions_LIBRARIES}
                      ${kdl_parser_LIBRARIES}
                      ${wbc_solvers-hls_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_link_libraries(test_kinematic_robot_model_kdl
                      wbc
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
