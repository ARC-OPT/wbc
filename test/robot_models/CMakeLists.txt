find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src)

add_subdirectory(kdl)
if(USE_HYRODYN)
    add_subdirectory(hyrodyn)
    add_subdirectory(rbdl)
endif()
if(USE_PINOCCHIO)
    add_subdirectory(pinocchio)
endif()


if(USE_HYRODYN AND USE_PINOCCHIO)

    pkg_search_module(base-types REQUIRED base-types)
    pkg_search_module(qpOASES REQUIRED qpOASES)
    pkg_search_module(orocos-kdl REQUIRED orocos-kdl)
    pkg_search_module(kdl_parser REQUIRED kdl_parser)
    pkg_search_module (hyrodyn REQUIRED hyrodyn)

    include_directories(${base-types_INCLUDE_DIRS}
                        ${qpOASES_INCLUDE_DIRS}
                        ${orocos-kdl_INCLUDE_DIRS}
                        ${kdl_parser_INCLUDE_DIRS}
                        ${hyrodyn_INCLUDE_DIR})
    link_directories(${base-types_LIBRARY_DIRS}
                     ${qpOASES_LIBRARY_DIRS}
                     ${orocos-kdl_LIBRARY_DIRS}
                     ${kdl_parser_LIBRARY_DIRS}
                     ${hyrodyn_LIBRARY_DIRS})

   add_executable(test_model_consistency test_model_consistency.cpp ../suite.cpp  test_robot_model.cpp)
   target_link_libraries(test_model_consistency
                         wbc-robot_models-hyrodyn
                         wbc-robot_models-kdl
                         wbc-robot_models-rbdl
                         wbc-robot_models-pinocchio
                         ${base-types_LIBRARIES}
                         ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
endif()
