find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

pkg_search_module(base-types REQUIRED base-types)
pkg_search_module(pinocchio REQUIRED pinocchio)
pkg_search_module(kdl_parser REQUIRED kdl_parser)

include_directories(${base-types_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS} ${kdl_parser_INCLUDE_DIRS})
link_directories(${base-types_LIBRARY_DIRS} ${pinocchio_LIBRARY_DIRS} ${kdl_parser_LIBRARY_DIRS})

add_executable(test_robot_model_pinocchio test_robot_model_pinocchio.cpp ../../suite.cpp)
target_link_libraries(test_robot_model_pinocchio
                      wbc-robot_models-pinocchio
                      wbc-robot_models-hyrodyn
                      wbc-robot_models-kdl
                      ${base-types_LIBRARIES}
                      ${kdl_parser_LIBRARIES}
                      ${pinocchio_LIBRARIES}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(compare_hyordyn_vs_pinocchio compare_hyordyn_vs_pinocchio.cpp ../../suite.cpp)
target_link_libraries(compare_hyordyn_vs_pinocchio
                    wbc-robot_models-pinocchio
                    wbc-robot_models-hyrodyn
                    ${base-types_LIBRARIES}
                    ${pinocchio_LIBRARIES}
                    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})


find_package(RBDL COMPONENTS URDFReader REQUIRED)
add_executable(compare_all compare_all.cpp)
target_link_libraries(compare_all
                    wbc-robot_models-pinocchio
                    wbc-robot_models-hyrodyn
                    wbc-robot_models-kdl
                    ${RBDL_LIBRARY}
                    ${RBDL_URDFReader_LIBRARY}
                    ${base-types_LIBRARIES}
                    ${pinocchio_LIBRARIES})
add_executable(compare_all_floating_base compare_all_floating_base.cpp)
target_link_libraries(compare_all_floating_base
                    wbc-robot_models-pinocchio
                    wbc-robot_models-hyrodyn
                    wbc-robot_models-kdl
                    ${RBDL_LIBRARY}
                    ${RBDL_URDFReader_LIBRARY}
                    ${base-types_LIBRARIES}
                    ${pinocchio_LIBRARIES})
