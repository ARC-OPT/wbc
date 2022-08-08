set(TARGET_NAME wbc-robot_models-rbdl)

FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)

file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models/rbdl "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models/rbdl "*.hpp")

pkg_search_module(rbdl REQUIRED rbdl)
pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(urdfdom REQUIRED urdfdom)

list(APPEND PKGCONFIG_REQUIRES wbc-core)
list(APPEND PKGCONFIG_REQUIRES rbdl)
list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES urdfdom)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

include_directories(${rbdl_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS} ${base-types_INCLUDE_DIRS})
link_directories(${urdfdom_LIBRARY_DIRS})
link_directories(${rbdl_LIBRARY_DIRS})
link_directories(${base-types_LIBRARY_DIRS})
link_directories(${urdfdom_LIBRARY_DIRS})

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME}
                      wbc-core
                      ${urdfdom_LIBRARIES}
                      ${base-types_LIBRARIES}
                      ${rbdl_LIBRARIES}
                      ${URDFReader_LIBRARIES})

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/wbc/robot_models/rbdl)
