rock_library(wbc
    SOURCES Wbc.cpp HierarchicalWDLSSolver.cpp TaskFrame.cpp RobotModel.cpp
    HEADERS Wbc.hpp HierarchicalWDLSSolver.hpp TaskFrame.hpp RobotModel.hpp WbcTypes.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
