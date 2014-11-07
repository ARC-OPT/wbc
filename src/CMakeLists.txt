rock_library(wbc
    SOURCES HierarchicalWDLSSolver.cpp
            GeneralizedInverse.cpp
            ConstraintConfig.hpp
            Constraint.cpp
            ExtendedConstraint.hpp
            KinematicChainKDL.cpp
            RobotModelKDL.cpp
            WbcVelocity.cpp
    HEADERS HierarchicalWDLSSolver.hpp
            GeneralizedInverse.hpp
            ConstraintConfig.hpp
            Constraint.hpp
            ExtendedConstraint.hpp
            KinematicChainKDL.hpp
            TaskFrameKDL.hpp
            RobotModelKDL.hpp
            WbcVelocity.hpp
            SolverTypes.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib kdl_conversions)
