rock_library(wbc
    SOURCES Wbc.cpp
            WbcVelocity.cpp
            solvers/Solver.hpp
            solvers/HierarchicalLeastSquaresSolver.cpp
            common/ConstraintConfig.cpp
            common/Constraint.cpp
            common/KinematicConstraintKDL.hpp
            common/TaskFrame.hpp
            common/TaskFrameKDL.cpp
            common/OptProblem.hpp
            robot_models/RobotModel.cpp
            robot_models/KinematicRobotModelKDL.cpp
            robot_models/RobotModelConfig.hpp
    HEADERS Wbc.hpp
            WbcVelocity.hpp
            solvers/Solver.hpp
            solvers/HierarchicalLeastSquaresSolver.hpp
            common/ConstraintConfig.hpp
            common/Constraint.hpp
            common/KinematicConstraintKDL.hpp
            common/TaskFrame.hpp
            common/TaskFrameKDL.hpp
            common/OptProblem.hpp
            robot_models/RobotModel.hpp
            robot_models/KinematicRobotModelKDL.hpp
            robot_models/RobotModelConfig.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib kdl_conversions kdl_parser)
