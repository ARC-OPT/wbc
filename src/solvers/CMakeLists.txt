set(HEADERS qp_solver.hpp)

add_subdirectory(qpoases)
add_subdirectory(hls)

INSTALL(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME}/solvers)
