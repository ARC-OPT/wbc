file(GLOB WBC_CORE_SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/core
    "*.cpp"
)

file(GLOB WBC_CORE_HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/core
    "*.hpp"
)

pkg_search_module(BASE_TYPES REQUIRED base-types)
pkg_search_module(BASE_LOGGING REQUIRED base-logging)

include_directories(${BASE_TYPES_INCLUDE_DIRS}
                 ${BASE_LOGGING_INCLUDE_DIRS})
link_directories(${BASE_TYPES_LIBRARY_DIRS}
                 ${BASE_LOGGING_LIBRARY_DIRS})

add_library(wbc-core SHARED ${WBC_CORE_SOURCES} ${WBC_CORE_HEADERS})
target_link_libraries(wbc-core ${BASE_TYPES_LIBRARIES} ${BASE_LOGGING_LIBRARIES})

install(TARGETS wbc-core
        LIBRARY DESTINATION lib)
set_target_properties(wbc-core PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${API_VERSION})

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/wbc-core.pc.in ${CMAKE_CURRENT_BINARY_DIR}/wbc-core.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wbc-core.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${WBC_CORE_HEADERS} DESTINATION include/${PROJECT_NAME}/core)
