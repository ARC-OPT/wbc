rock_library(wbc
    SOURCES HierarchicalWDLSSolver.cpp
    HEADERS HierarchicalSolver.hpp HierarchicalWDLSSolver.hpp Priority.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
