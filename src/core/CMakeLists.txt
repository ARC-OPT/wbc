file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/core "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/core "*.hpp")

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(base-logging REQUIRED base-logging)

include_directories(${base-types_INCLUDE_DIRS} ${base-logging_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${base-logging_LIBRARY_DIRS})

add_library(wbc-core SHARED ${SOURCES} ${HEADERS})
target_link_libraries(wbc-core ${base-types_LIBRARIES} ${base-logging_LIBRARIES})

install(TARGETS wbc-core
        LIBRARY DESTINATION lib)
set_target_properties(wbc-core PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${API_VERSION})

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/wbc-core.pc.in ${CMAKE_CURRENT_BINARY_DIR}/wbc-core.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wbc-core.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/core)
