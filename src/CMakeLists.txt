rock_library(wbc
    SOURCES HierarchicalWDLSSolver.cpp Wbc.cpp TaskFrame.cpp RobotModel.cpp
    HEADERS HierarchicalWDLSSolver.hpp SubTask.hpp Wbc.hpp HierarchicalSolver.hpp TaskFrame.hpp RobotModel.hpp WbcTypes.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
