#ifndef REACHABLE_GOALSUBSETS_CEGAR_PRUNING_H
#define REACHABLE_GOALSUBSETS_CEGAR_PRUNING_H

#include "../../pruning_method.h"
#include "../../heuristic.h"
#include "../../cegar/additive_cartesian_heuristic.h"
#include "msgs_collection.h"
#include "../goal_subsets/goal_subset.h"

#include <memory>

namespace reachable_goal_subsets_cegar_pruning {

class ReachableGoalSubsetsCegarPruning : public PruningMethod {

    std::shared_ptr<Evaluator> h;

    std::shared_ptr<cegar::AdditiveCartesianHeuristic> cegar_heuristic;
    MSGSCollection current_msgs;

    bool initialized = false;

    bool prune(const State &state, int remaining_cost) override;
    void prune(const State &, std::vector<OperatorID> &) override {};


public:
    explicit ReachableGoalSubsetsCegarPruning(const options::Options &opts);
    virtual void initialize(const std::shared_ptr<AbstractTask> &) override;
    virtual void print_statistics() const override;
    
    virtual MSGSCollection get_msgs() const override;
    virtual void init_msgs(MSGSCollection goals) override;
};

}

#endif
