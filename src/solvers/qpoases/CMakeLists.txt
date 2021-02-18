SET(TARGET_NAME wbc-solvers-qpoases)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(qpOASES REQUIRED qpOASES)

set(SOURCES QPOasesSolver.cpp)
set(HEADERS QPOasesSolver.hpp)

list(APPEND PKGCONFIG_REQUIRES qpOASES)
list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES wbc-core)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

include_directories(${base-types_INCLUDE_DIRS}
                    ${qpOASES_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS}
                 ${qpOASES_LIBRARY_DIRS})

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME}
                      wbc-core
                      ${base-types_LIBRARIES}
                      ${qpOASES_LIBRARIES})

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/solvers/qpoases)
