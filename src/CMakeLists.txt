rock_library(wbc
    SOURCES WbcVelocity.cpp HierarchicalWDLSSolver.cpp TaskFrame.cpp SubTaskConfig.hpp SubTask.cpp ExtendedSubTask.hpp PriorityData.hpp
    HEADERS WbcVelocity.hpp HierarchicalWDLSSolver.hpp TaskFrame.hpp SubTaskConfig.hpp SubTask.hpp ExtendedSubTask.hpp PriorityData.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
