#ifndef TASKFRAME_HPP
#define TASKFRAME_HPP

#include <base/samples/joints.h>

/**
 * @brief The TaskFrame class Only storage of pose, wrench, etc. or also include solvers?
 */
class TaskFrame{
public:
    TaskFrame();
    ~TaskFrame();

    void Update();
};

#endif
