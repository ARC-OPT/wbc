rock_library(wbc
    SOURCES HierarchicalWDLSSolver.cpp
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
