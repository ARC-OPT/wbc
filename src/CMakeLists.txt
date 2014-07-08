rock_library(wbc
    SOURCES HierarchicalWDLSSolver.cpp
            TaskFrameKDL.cpp
            ConstraintConfig.hpp
            Constraint.cpp
            ExtendedConstraint.hpp
            PriorityData.hpp
            RobotModelKDL.cpp
    HEADERS HierarchicalWDLSSolver.hpp
            TaskFrameKDL.hpp
            ConstraintConfig.hpp
            Constraint.hpp
            ExtendedConstraint.hpp
            PriorityData.hpp
            TaskFrame.hpp
            RobotModelKDL.hpp
            WbcVelocity.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
