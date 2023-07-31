#ifndef FAST_DOWNWARD_TASK_RELAXATION_TRACKER_H
#define FAST_DOWNWARD_TASK_RELAXATION_TRACKER_H

#include <string>
#include <iostream>

#include <unordered_map>

#include "../../task_proxy.h"
#include "relaxed_task.h"

using namespace std;

class TaskRelaxationTracker {

private:
    int current_index = -1;
    std::vector<RelaxedTask*> relaxed_tasks;

    bool internal_relaxation;
    bool external_relaxation;


public:
    TaskRelaxationTracker(const std::shared_ptr<AbstractTask> task);

    std::vector<RelaxedTask*> get_relaxed_tasks() const {return relaxed_tasks;}
    bool has_next_relaxed_task();
    RelaxedTask* next_relaxed_task();
    RelaxedTask* current_relaxed_task();
};


#endif //FAST_DOWNWARD_TASK_RELAXATION_TRACKER_H
