rock_library(wbc
    SOURCES WbcVelocity.cpp HierarchicalWDLSSolver.cpp TaskFrame.cpp RobotModel.cpp
    HEADERS WbcVelocity.hpp HierarchicalWDLSSolver.hpp TaskFrame.hpp RobotModel.hpp SubTaskConfig.hpp SubTask.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
