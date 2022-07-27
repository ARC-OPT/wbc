FIND_PACKAGE( Boost COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(joint_limits_from_urdf joint_limits_from_urdf.cpp)
target_link_libraries(joint_limits_from_urdf wbc-tools ${Boost_LIBRARIES})

install(TARGETS joint_limits_from_urdf
        LIBRARY RUNTIME DESTINATION bin)


