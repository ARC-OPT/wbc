set(TARGET_NAME wbc-robot_models-pinocchio)

file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models/robot_model_pinocchio "*.cpp")
file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models/robot_model_pinocchio "*.hpp")

pkg_search_module(pinocchio REQUIRED IMPORTED_TARGET pinocchio)

list(APPEND PKGCONFIG_REQUIRES wbc-core)
list(APPEND PKGCONFIG_REQUIRES pinocchio)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME} PUBLIC
                      wbc-core
                      PkgConfig::pinocchio)

set_target_properties(${TARGET_NAME} PROPERTIES
       VERSION ${PROJECT_VERSION}
       SOVERSION ${API_VERSION})

install(TARGETS ${TARGET_NAME}
        LIBRARY DESTINATION lib)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${HEADERS} DESTINATION include/wbc/robot_models/pinocchio)

