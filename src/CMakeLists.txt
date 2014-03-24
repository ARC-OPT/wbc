rock_library(wbc
    SOURCES WbcVelocity.cpp HierarchicalWDLSSolver.cpp TaskFrame.cpp SubTaskConfig.hpp SubTask.hpp ExtendedSubTask.hpp
    HEADERS WbcVelocity.hpp HierarchicalWDLSSolver.hpp TaskFrame.hpp SubTaskConfig.hpp SubTask.hpp ExtendedSubTask.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
