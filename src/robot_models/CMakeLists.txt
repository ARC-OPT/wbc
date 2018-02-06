file(GLOB WBC_ROBOT_MODEL_SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models
    "*.cpp"
)

file(GLOB WBC_ROBOT_MODEL_HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/robot_models
    "*.hpp"
)

pkg_search_module(KDL_PARSER REQUIRED kdl_parser)

include_directories(${KDL_PARSER_INCLUDE_DIRS})
link_directories(${KDL_PARSER_LIBRARY_DIRS})

add_library(wbc-robot_models SHARED ${WBC_ROBOT_MODEL_SOURCES} ${WBC_ROBOT_MODEL_HEADERS})
target_link_libraries(wbc-robot_models wbc-core ${KDL_PARSER_LIBRARIES})

install(TARGETS wbc-robot_models
        LIBRARY DESTINATION lib)
set_target_properties(wbc-robot_models PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${API_VERSION})

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/wbc-robot_models.pc.in ${CMAKE_CURRENT_BINARY_DIR}/wbc-robot_models.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wbc-robot_models.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${WBC_ROBOT_MODEL_HEADERS} DESTINATION include/${PROJECT_NAME}/robot_models)

