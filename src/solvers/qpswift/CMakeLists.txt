SET(TARGET_NAME wbc-solvers-qpswift)

pkg_search_module(qpSWIFT REQUIRED IMPORTED_TARGET qpSWIFT)

set(SOURCES QPSwiftSolver.cpp)
set(HEADERS QPSwiftSolver.hpp)

list(APPEND PKGCONFIG_REQUIRES qpSWIFT)
list(APPEND PKGCONFIG_REQUIRES wbc-core)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME} PUBLIC
                      wbc-core
                      PkgConfig::qpSWIFT)

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/solvers/qpswift)

add_subdirectory(test)
