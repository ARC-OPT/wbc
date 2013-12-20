rock_library(wbc
    SOURCES HierarchicalLSSolver.cpp
    HEADERS HierarchicalSolver.hpp HierarchicalLSSolver.hpp Priority.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
