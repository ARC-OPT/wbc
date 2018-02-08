include_directories(.)

file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src "*.hpp")

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(base-logging REQUIRED base-logging)
pkg_search_module(kdl_parser REQUIRED kdl_parser)
pkg_search_module(kdl_conversions REQUIRED kdl_conversions)

list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES base-logging)
list(APPEND PKGCONFIG_REQUIRES kdl_parser)
list(APPEND PKGCONFIG_REQUIRES kdl_conversions)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

include_directories(${base-types_INCLUDE_DIRS}
                    ${base-logging_INCLUDE_DIRS}
                    ${kdl_parser_INCLUDE_DIRS}
                    ${kdl_conversions_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${base-logging_LIBRARY_DIRS}
                 ${kdl_parser_LIBRARY_DIRS}
                 ${kdl_conversions_LIBRARY_DIRS})

set(TARGET_NAME ${PROJECT_NAME})

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME}
                      ${base-types_LIBRARIES}
                      ${base-logging_LIBRARIES}
                      ${kdl_parser_LIBRARIES}
                      ${kdl_conversions_LIBRARIES})

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${TARGET_NAME})
