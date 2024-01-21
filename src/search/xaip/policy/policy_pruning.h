#ifndef PRUNING_POLICY_METHOD_H
#define PRUNING_POLICY_METHOD_H

#include "../../pruning_method.h"
#include "../../utils/timer.h"
#include "distance_function.h"

#include <unordered_map>

namespace policy_pruning_method {
class PolicyPruningMethod : public PruningMethod {

    float radius;
    std::shared_ptr<DistanceFunction> distance_function;

    virtual void prune(const State &, std::vector<OperatorID> &) override;
public:
    explicit PolicyPruningMethod(const options::Options &opts);
    virtual void initialize(const std::shared_ptr<AbstractTask> &) override;
    virtual void notify_initial_state(const State & initial_state) override;
    virtual void notify_state_transition(const State & parent_state, OperatorID op_id, const State & state);
    virtual void print_statistics() const override;

    virtual std::vector<std::pair<float,OperatorID>> postpone(const State &, std::vector<OperatorID> &);

    void set_radius(float radius) {
        this->radius = radius;
    }
};
}

#endif
