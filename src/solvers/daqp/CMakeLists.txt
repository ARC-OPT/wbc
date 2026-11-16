SET(TARGET_NAME wbc-solvers-daqp)

# DAQP (https://github.com/darnstrom/daqp) does not provide a pkg-config file
find_path(DAQP_INCLUDE_DIR NAMES daqp/api.h)
find_library(DAQP_LIBRARY NAMES daqp)
if(NOT DAQP_INCLUDE_DIR OR NOT DAQP_LIBRARY)
    message(FATAL_ERROR "DAQP not found. Install it from https://github.com/darnstrom/daqp or set CMAKE_PREFIX_PATH")
endif()

set(SOURCES DaqpSolver.cpp)
set(HEADERS DaqpSolver.hpp)

list(APPEND PKGCONFIG_REQUIRES wbc-core)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")
get_filename_component(DAQP_LIBRARY_DIR ${DAQP_LIBRARY} DIRECTORY)
set(PKGCONFIG_LIBS "-L${DAQP_LIBRARY_DIR} -ldaqp")
set(PKGCONFIG_CFLAGS "-I${DAQP_INCLUDE_DIR}")

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(${TARGET_NAME} PUBLIC ${DAQP_INCLUDE_DIR})
target_link_libraries(${TARGET_NAME} PUBLIC
                      wbc-core
                      ${DAQP_LIBRARY})

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/solvers/daqp)

add_subdirectory(test)
