pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(kdl_conversions REQUIRED kdl_conversions)

include_directories(${base-types_INCLUDE_DIRS} ${kdl_conversions_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${kdl_conversions_LIBRARY_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_executable(test_wbc-tools test_wbc-tools.cpp ../suite.cpp)
target_link_libraries(test_wbc-tools wbc-tools
                      ${kdl_conversions_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
