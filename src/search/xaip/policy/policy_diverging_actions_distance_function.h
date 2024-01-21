#ifndef POLICY_DIVERGING_ACTIONS_DISTANCE_FUNCTION_METHOD_H
#define POLICY_DIVERGING_ACTIONS_DISTANCE_FUNCTION_METHOD_H

#include "distance_function.h"
#include "policy_client.h"
#include "../../utils/timer.h"

#include <unordered_map>

namespace policy_distance_function {
class PolicyDivergingActionsDistanceFunction : public DistanceFunction {

    std::string url;
    bool project_resources = true;
    policy::PolicyClient policy_client;

    std::unordered_map<StateID,int> divergence_count;
    std::unordered_map<StateID,StateID> parent_map;
    std::unordered_map<StateID,OperatorID> policy_action;

    utils::Timer policy_evaluation_timer;

public:
    explicit PolicyDivergingActionsDistanceFunction(const options::Options &opts);
    std::vector<float> get_distances(const State &state, std::vector<OperatorID> &op_ids) override;
    virtual void initialize(const std::shared_ptr<AbstractTask> &) override;
    virtual void notify_initial_state(const State & initial_state) override;
    virtual void notify_state_transition(const State & parent_state, OperatorID op_id, const State & state);
    virtual void print_statistics() const override;
};
}

#endif
