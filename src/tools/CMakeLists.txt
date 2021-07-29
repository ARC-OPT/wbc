set(TARGET_NAME wbc-tools)

file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/types "*.hpp")
file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/types "*.cpp")

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(base-logging REQUIRED base-logging)
pkg_search_module(urdfdom REQUIRED urdfdom)

list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES base-logging)
list(APPEND PKGCONFIG_REQUIRES urdfdom)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

include_directories(${base-types_INCLUDE_DIRS} ${base-logging_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${base-logging_LIBRARY_DIRS} ${urdfdom_LIBRARY_DIRS})

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME}
                      ${base-types_LIBRARIES}
                      ${base-logging_LIBRARIES}
                      ${urdfdom_LIBRARIES})

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/wbc/tools)
