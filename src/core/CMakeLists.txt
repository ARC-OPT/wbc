set(TARGET_NAME wbc-core)

file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/core "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/core "*.hpp")

list(APPEND PKGCONFIG_REQUIRES wbc-tools)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME} PUBLIC
                      wbc-tools
                      wbc-types
                      dl)

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/wbc/core)

add_subdirectory(test)
