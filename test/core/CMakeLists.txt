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

add_executable(test_core test_core.cpp ../suite.cpp)
target_link_libraries(test_core
                      wbc-core
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
