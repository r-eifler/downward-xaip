#ifndef TASKS_MODIFIED_INIT_TASK_H
#define TASKS_MODIFIED_INIT_TASK_H

#include "../../tasks/delegating_task.h"
#include <vector>

namespace extra_tasks {
class ModifiedInitTask : public tasks::DelegatingTask {
    std::vector<int> init;

public:
    ModifiedInitTask(
        const std::shared_ptr<AbstractTask> &parent, std::vector<FactPair> &&change_init);
    ~ModifiedInitTask() = default;

    virtual std::vector<int> get_initial_state_values() const override;
};
}

#endif
