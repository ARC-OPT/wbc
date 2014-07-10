rock_library(wbc
    SOURCES HierarchicalWDLSSolver.cpp
            TaskFrame.cpp
            ConstraintConfig.hpp
            Constraint.cpp
            ExtendedConstraint.hpp
            KinematicChainKDL.cpp
            PriorityData.hpp
            RobotModelKDL.cpp
            WbcVelocity.cpp
    HEADERS HierarchicalWDLSSolver.hpp
            ConstraintConfig.hpp
            Constraint.hpp
            ExtendedConstraint.hpp
            PriorityData.hpp
            SolverTypes.hpp
            KinematicChainKDL.cpp
            TaskFrame.hpp
            RobotModelKDL.hpp
            WbcVelocity.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib kdl_conversions)
