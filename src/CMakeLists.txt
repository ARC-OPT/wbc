rock_library(wbc
    SOURCES Wbc.cpp
            WbcVelocity.cpp
            solvers/Solver.hpp
            solvers/HierarchicalLeastSquaresSolver.cpp
            constraints/ConstraintConfig.hpp
            constraints/Constraint.cpp
            constraints/ExtendedConstraint.hpp
            models/TaskFrame.hpp
            models/TaskFrameKDL.cpp
            models/RobotModel.cpp
            models/KinematicRobotModelKDL.cpp
            solvers/OptProblem.hpp
    HEADERS Wbc.hpp
            WbcVelocity.hpp
            solvers/Solver.hpp
            solvers/HierarchicalLeastSquaresSolver.hpp
            constraints/ConstraintConfig.hpp
            constraints/Constraint.hpp
            constraints/ExtendedConstraint.hpp
            solvers/OptProblem.hpp
            models/TaskFrame.hpp
            models/TaskFrameKDL.hpp
            models/RobotModel.hpp
            models/KinematicRobotModelKDL.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib kdl_conversions)
