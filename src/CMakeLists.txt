rock_library(wbc
    SOURCES HierarchicalWDLSSolver.cpp SubTask.cpp Wbc.cpp
    HEADERS HierarchicalWDLSSolver.hpp SubTask.hpp Wbc.hpp HierarchicalSolver.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
