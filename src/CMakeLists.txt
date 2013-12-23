rock_library(wbc
    SOURCES HierarchicalLSSolver.cpp TaskFrame.cpp
    HEADERS HierarchicalSolver.hpp HierarchicalLSSolver.hpp Priority.hpp TaskFrame.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
