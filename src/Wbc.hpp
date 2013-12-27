#ifndef WBC_HPP
#define WBC_HPP

#include <kdl/tree.hpp>
#include <base/commands/joints.h>

class HierarchicalSolver;
class SubTask;

typedef std::map<std::string, HierarchicalSolver*> SolverMap;
typedef std::map<std::string, int> JointIndexMap;

class Wbc{

    SolverMap solver_map_;
    std::vector<SubTask*> sub_task_map_;
    JointIndexMap joint_index_map_;
    bool configured_;
public:
    Wbc(){configured_ = false;}
    ~Wbc(){}

    bool AddSubTask(const KDL::Chain &chain,
                    const uint no_task_variables,
                    const uint priority);

    bool AddSolver(const std::string &name,
                   HierarchicalSolver* solver);

    bool Configure();

    void Solve(const std::string &solver_name,
               const base::samples::Joints& status,
               base::commands::Joints& solver_output);
};

#endif

