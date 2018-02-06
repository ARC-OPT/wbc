file(GLOB WBC_SOLVERS_SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src/solvers
    "*.cpp"
)

file(GLOB WBC_SOLVERS_HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src/solvers
    "*.hpp"
)

pkg_search_module(BASE_TYPES REQUIRED base-types)
include_directories(${BASE_TYPES_INCLUDE_DIRS})
link_directories(${BASE_TYPES_LIBRARY_DIRS})

add_library(wbc-solvers SHARED ${WBC_SOLVERS_SOURCES} ${WBC_SOLVERS_HEADERS})
target_link_libraries(wbc-solvers wbc-core)

install(TARGETS wbc-solvers
        LIBRARY DESTINATION lib)
set_target_properties(wbc-solvers PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${API_VERSION})

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/wbc-solvers.pc.in ${CMAKE_CURRENT_BINARY_DIR}/wbc-solvers.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wbc-solvers.pc DESTINATION lib/pkgconfig)
INSTALL(FILES ${WBC_SOLVERS_HEADERS} DESTINATION include/${PROJECT_NAME}/solvers)


