cmake_minimum_required (VERSION 3.16.3)
project(wbc)

find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem unit_test_framework serialization)

set(PROJECT_VERSION 0.2)
set(API_VERSION ${PROJECT_VERSION})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

option(ROBOT_MODEL_RBDL "Also build the RBDL-based robot model, by default only pinocchio is built" OFF)
option(ROBOT_MODEL_KDL "Also build the KDL-based robot model, by default only pinocchio is built" OFF)
option(ROBOT_MODEL_HYRODYN "Also build the HyRoDyn-based robot model, by default only pinocchio is built" OFF)
option(SOLVER_PROXQP "Build the ProxQP-based solver, by default only hls and qpoases are built" OFF)
option(SOLVER_EIQUADPROG "Build the Eiquadprog-based solver, by default only hls and qpoases are built" OFF)
option(SOLVER_QPSWIFT "Build the QPSwift-based solver, by default only hls and qpoases are built" OFF)

add_subdirectory(src)
add_subdirectory(tutorials)

find_package(Doxygen)

if(DOXYGEN_FOUND)
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${doxyfile_in} ${doxyfile} @ONLY)
    message("Doxygen build started.")
    add_custom_target(doc
                      COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile_in}
                      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                      COMMENT "Generating API documentation with Doxygen"
                      VERBATIM)
else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)
