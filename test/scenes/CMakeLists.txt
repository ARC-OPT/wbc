find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(ctrl_types REQUIRED ctrl_types)
include_directories(${base-types_INCLUDE_DIRS} ${ctrl_types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${ctrl_types_LIBRARY_DIRS})

add_executable(wbc_velocity_scene wbc_velocity_scene.cpp ../suite.cpp)
target_link_libraries(wbc_velocity_scene
                      wbc-scenes
                      wbc-robot_models
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
