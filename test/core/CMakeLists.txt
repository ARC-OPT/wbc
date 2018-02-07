pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(kdl REQUIRED orocos-kdl)

include_directories(${kdl_INCLUDE_DIRS} ${base-types_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../src)
link_directories(${kdl_LIBRARY_DIRS} ${base-types_LIBRARY_DIRS})

add_executable(test_wbc-core test_wbc-core.cpp ../suite.cpp)
target_link_libraries(test_wbc-core 
                      wbc-core
                      ${kdl_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
