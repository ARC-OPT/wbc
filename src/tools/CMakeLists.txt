file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/tools "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/tools "*.hpp")

pkg_search_module(base-types REQUIRED base-types)

include_directories(${base-types_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS})

add_library(wbc-tools SHARED ${SOURCES} ${HEADERS})
target_link_libraries(wbc-tools ${base-types_LIBRARIES})

install(TARGETS wbc-tools
        LIBRARY DESTINATION lib)
set_target_properties(wbc-tools PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${API_VERSION})

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/wbc-tools.pc.in ${CMAKE_CURRENT_BINARY_DIR}/wbc-tools.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wbc-tools.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/tools)

