rock_library(wbc
    SOURCES HierarchicalWDLSSolver.cpp
            ConstraintConfig.hpp
            Constraint.cpp
            ExtendedConstraint.hpp
            WbcVelocity.cpp
            TaskFrame.cpp
            KinematicModel.cpp
            LinearEquationSystem.hpp
    HEADERS HierarchicalWDLSSolver.hpp
            ConstraintConfig.hpp
            Constraint.hpp
            ExtendedConstraint.hpp
            WbcVelocity.hpp
            LinearEquationSystem.hpp
            TaskFrame.hpp
            KinematicModel.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib kdl_conversions)
