rock_library(wbc
    SOURCES WbcVelocity.cpp HierarchicalWDLSSolver.cpp TaskFrame.cpp ConstraintConfig.hpp Constraint.cpp ExtendedConstraint.hpp PriorityData.hpp
    HEADERS WbcVelocity.hpp HierarchicalWDLSSolver.hpp TaskFrame.hpp ConstraintConfig.hpp Constraint.hpp ExtendedConstraint.hpp PriorityData.hpp
    DEPS_PKGCONFIG orocos-kdl base-lib)
