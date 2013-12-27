rock_library(wbc
    SOURCES HierarchicalSolver.cpp SubTask.cpp Wbc.cpp
    HEADERS HierarchicalSolver.hpp SubTask.hpp Wbc.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
