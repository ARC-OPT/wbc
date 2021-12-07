set(TARGET_NAME wbc-robot_models-kdl)

file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models/robot_model_kdl "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models/robot_model_kdl "*.hpp")

pkg_search_module(kdl_parser REQUIRED kdl_parser)

list(APPEND PKGCONFIG_REQUIRES kdl_parser)
list(APPEND PKGCONFIG_REQUIRES wbc-core)
list(APPEND PKGCONFIG_REQUIRES orocos-kdl)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

include_directories(${kdl_parser_INCLUDE_DIRS} ${orocos-kdl_INCLUDE_DIRS})
link_directories(${kdl_parser_LIBRARY_DIRS} ${orocos-kdl_LIBRARY_DIRS})

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME}
                      wbc-core
                      ${kdl_parser_LIBRARIES}
                      ${orocos-kdl_LIBRARIES})

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/wbc/robot_models/kdl)
