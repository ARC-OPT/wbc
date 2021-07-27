pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(base-logging REQUIRED base-logging)
pkg_search_module(urdfdom REQUIRED urdfdom)

FIND_PACKAGE( Boost COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

include_directories($CMAKE_CURRENT_SOURCE_DIR}/src ${base-types_INCLUDE_DIRS} ${base-logging_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${base-logging_LIBRARY_DIRS} ${urdfdom_LIBRARY_DIRS})

add_executable(joint_limits_from_urdf joint_limits_from_urdf.cpp)
target_link_libraries(joint_limits_from_urdf wbc-tools ${Boost_LIBRARIES})

install(TARGETS joint_limits_from_urdf
        LIBRARY RUNTIME DESTINATION bin)


