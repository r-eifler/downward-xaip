#ifndef REACHABLE_GOALSUBSETS_HMAX_PRUNING_H
#define REACHABLE_GOALSUBSETS_HMAX_PRUNING_H

#include "../../pruning_method.h"
#include "../../heuristic.h"
#include "../../heuristics/max_heuristic.h"
#include "msgs_collection.h"
#include "../goal_subsets/goal_subset.h"

#include <memory>

namespace reachable_goal_subsets_tracking {

class ReachableGoalSubsetsTracking : public PruningMethod {

    MSGSCollection current_msgs;

    bool prune(const State &state, int remaining_cost) override;
    void prune(const State &, std::vector<OperatorID> &) override {};


public:
    explicit ReachableGoalSubsetsTracking(const options::Options &opts);
    virtual void initialize(const std::shared_ptr<AbstractTask> &) override;
    virtual void print_statistics() const override;
};

}

#endif