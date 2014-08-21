rock_library(wbc
    SOURCES HierarchicalWDLSSolver.cpp
            HierarchicalDCPISolver.cpp
            GeneralizedInverse.cpp
            TaskFrame.cpp
            ConstraintConfig.hpp
            Constraint.cpp
            ExtendedConstraint.hpp
            KinematicChainKDL.cpp
            PriorityData.hpp
            RobotModelKDL.cpp
            WbcVelocity.cpp
    HEADERS HierarchicalWDLSSolver.hpp
            HierarchicalDCPISolver.hpp
            GeneralizedInverse.hpp
            ConstraintConfig.hpp
            Constraint.hpp
            ExtendedConstraint.hpp
            PriorityData.hpp
            SolverTypes.hpp
            KinematicChainKDL.hpp
            TaskFrame.hpp
            RobotModelKDL.hpp
            WbcVelocity.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib kdl_conversions)
