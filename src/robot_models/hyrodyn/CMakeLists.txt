set(TARGET_NAME wbc-robot_models-hyrodyn)

file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models/robot_model_hyrodyn "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models/robot_model_hyrodyn "*.hpp")

pkg_search_module(urdfdom REQUIRED urdfdom)
pkg_search_module(hyrodyn REQUIRED hyrodyn)
pkg_search_module(base-types REQUIRED base-types)

list(APPEND PKGCONFIG_REQUIRES wbc-core)
list(APPEND PKGCONFIG_REQUIRES urdfdom)
list(APPEND PKGCONFIG_REQUIRES base-types)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

include_directories(${urdfdom_INCLUDE_DIRS} ${hyrodyn_INCLUDE_DIRS} ${base-types_INCLUDE_DIRS})
link_directories(${urdfdom_LIBRARY_DIRS})
link_directories(${hyrodyn_LIBRARY_DIRS})
link_directories(${base-types_LIBRARY_DIRS})

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME}
                      wbc-core
                      ${base-types_LIBRARIES}
                      ${urdfdom_LIBRARIES}
                      ${hyrodyn_LIBRARIES})

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/wbc/robot_models/hyrodyn)
