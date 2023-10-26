#pragma once

#include "../../pruning_method.h"
#include "../../task_proxy.h"
#include "../../tasks/root_task.h"

#include <Python.h>

namespace policy_testing {
class ASNetInterface {
    PyObject *next_state_func;

public:
    ASNetInterface(
        const std::string &domain_pddl,
        const std::string &problem_pddl,
        const std::string &snapshot);
    ~ASNetInterface();

    OperatorID apply_policy(
        const State &state,
        const std::vector<OperatorID> &applicable_ops);
};

class ASNetsPolicyPruning : public PruningMethod {
    ASNetInterface policy;

public:
    explicit ASNetsPolicyPruning(options::Options &opts);
    void prune_operators(const State &state, std::vector<OperatorID> &op_ids) override;
    void print_statistics() const override;
};
} // namespace policy_testing
