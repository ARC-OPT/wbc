file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models "*.hpp")

pkg_search_module(kdl_parser REQUIRED kdl_parser)
pkg_search_module(kdl_conversions REQUIRED kdl_conversions)

include_directories(${kdl_parser_INCLUDE_DIRS} ${kdl_conversions_INCLUDE_DIRS})
link_directories(${kdl_parser_LIBRARY_DIRS} ${kdl_conversions_LIBRARY_DIRS})

add_library(wbc-robot_models SHARED ${SOURCES} ${HEADERS})
target_link_libraries(wbc-robot_models wbc-core wbc-tools ${kdl_parser_LIBRARIES} ${kdl_conversions_LIBRARIES})

install(TARGETS wbc-robot_models
        LIBRARY DESTINATION lib)
set_target_properties(wbc-robot_models PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${API_VERSION})

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/wbc-robot_models.pc.in ${CMAKE_CURRENT_BINARY_DIR}/wbc-robot_models.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wbc-robot_models.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/robot_models)

