rock_library(wbc
    SOURCES WbcVelocity.cpp HierarchicalWDLSSolver.cpp TaskFrame.cpp
    HEADERS WbcVelocity.hpp HierarchicalWDLSSolver.hpp TaskFrame.hpp SubTaskConfig.hpp SubTask.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
