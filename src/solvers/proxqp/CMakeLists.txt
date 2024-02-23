SET(TARGET_NAME wbc-solvers-proxqp)

pkg_search_module(proxsuite REQUIRED IMPORTED_TARGET proxsuite)

set(SOURCES ProxQPSolver.cpp)
set(HEADERS ProxQPSolver.hpp)

list(APPEND PKGCONFIG_REQUIRES proxsuite)
list(APPEND PKGCONFIG_REQUIRES wbc-core)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME} PUBLIC
                      wbc-core
                      PkgConfig::proxsuite)

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/solvers/proxqp)

add_subdirectory(test)
