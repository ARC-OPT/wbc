file(GLOB SOURCES RELATIVE ${PROJECT_SOURCE_DIR}/src
    "*.cpp"
)

file(GLOB HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/src
    "*.hpp"
)
rock_library(wbc
    SOURCES ${SOURCES} ${HEADERS}
    HEADERS ${HEADERS}
    DEPS_PKGCONFIG base-lib kdl_conversions kdl_parser)
