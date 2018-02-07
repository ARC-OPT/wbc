file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/solvers "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/solvers "*.hpp")

pkg_search_module(base-types REQUIRED base-types)

include_directories(${base-types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS})

add_library(wbc-solvers SHARED ${SOURCES} ${HEADERS})
target_link_libraries(wbc-solvers wbc-core wbc-tools)

install(TARGETS wbc-solvers
        LIBRARY DESTINATION lib)
set_target_properties(wbc-solvers PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${API_VERSION})

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/wbc-solvers.pc.in ${CMAKE_CURRENT_BINARY_DIR}/wbc-solvers.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wbc-solvers.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/solvers)


