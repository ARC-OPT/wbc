find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
pkg_search_module(kdl REQUIRED orocos-kdl)
pkg_search_module(wbc-core REQUIRED wbc-core)

include_directories(${kdl_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../src)
link_directories(${kdl_INCLUDE_DIRS} ${wbc-core_LIBRARY_DIRS})

add_executable(test_wbc test_wbc.cpp suite.cpp)
target_link_libraries(test_wbc wbc-core wbc-robot_models wbc-solvers wbc-scenes
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
