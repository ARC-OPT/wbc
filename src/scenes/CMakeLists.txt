set(TARGET_NAME wbc-scenes)

file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/scenes "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/scenes "*.hpp")

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(base-types REQUIRED ctrl_types)
list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES wbc-core)
list(APPEND PKGCONFIG_REQUIRES ctrl_types)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

include_directories(${base-types_INCLUDE_DIRS} ${ctrl_types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS})

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME}
                      wbc-core
                      ${ctrl_types_LIBRARIES}
                      ${base-types_LIBRARIES})

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/wbc/scenes)
