set(TARGET_NAME wbc-tools)

file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/types "*.hpp")
file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/types "*.cpp")

pkg_search_module(urdfdom REQUIRED IMPORTED_TARGET urdfdom)
pkg_search_module(eigen3 REQUIRED IMPORTED_TARGET eigen3)

list(APPEND PKGCONFIG_REQUIRES urdfdom)
list(APPEND PKGCONFIG_REQUIRES eigen3)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})

target_include_directories(${TARGET_NAME} PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>/src
	$<INSTALL_INTERFACE:include>)

target_link_libraries(${TARGET_NAME} PUBLIC
                      PkgConfig::urdfdom
                      PkgConfig::eigen3)

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/wbc/tools)
