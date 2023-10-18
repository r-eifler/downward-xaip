#ifndef PRUNING_POLICY_DIVERGING_ACTIONS_PRUNING_METHOD_H
#define PRUNING_POLICY_DIVERGING_ACTIONS_PRUNING_METHOD_H

#include "../../pruning_method.h"
#include "policy_client.h"
#include "../../utils/timer.h"

#include <unordered_map>

namespace policy_pruning_method {
class PolicyDivergingActionsPruningMethod : public PruningMethod {

    std::string url;
    bool project_resources = true;
    policy::PolicyClient policy_client;

    double max_diverging_actions = 1;

    std::unordered_map<StateID,int> divergence_count;
    std::unordered_map<StateID,StateID> parent_map;
    std::unordered_map<StateID,OperatorID> policy_action;

    utils::Timer policy_evaluation_timer;



    virtual void prune(const State &, std::vector<OperatorID> &) override;
public:
    explicit PolicyDivergingActionsPruningMethod(const options::Options &opts);
    virtual void initialize(const std::shared_ptr<AbstractTask> &) override;
    virtual void notify_initial_state(const State & initial_state) override;
    virtual void notify_state_transition(const State & parent_state, OperatorID op_id, const State & state);
    virtual void print_statistics() const override;
};
}

#endif
