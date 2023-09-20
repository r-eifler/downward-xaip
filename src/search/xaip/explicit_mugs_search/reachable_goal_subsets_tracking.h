#ifndef REACHABLE_GOALSUBSETS_HMAX_PRUNING_H
#define REACHABLE_GOALSUBSETS_HMAX_PRUNING_H

#include "../../pruning_method.h"
#include "../../heuristic.h"
#include "../../heuristics/max_heuristic.h"
#include "../explicit_mugs_search/msgs_collection.h"
#include "../goal_subsets/goal_subset.h"

#include <memory>

namespace reachable_goal_subsets_tracking {

class ReachableGoalSubsetsTracking : public PruningMethod {

    bool use_subset_dominance = false;

    MSGSCollection current_msgs;
    bool initialized = false;

    bool prune(const State &state, int remaining_cost) override;
    void prune(const State &, std::vector<OperatorID> &) override {};


public:
    explicit ReachableGoalSubsetsTracking(const options::Options &opts);
    virtual void initialize(const std::shared_ptr<AbstractTask> &) override;
    StateID get_cardinally_best_state() override {return current_msgs.get_cardinally_best_state();}
    int get_max_solved_soft_goals() override {return current_msgs.get_max_solved_soft_goals();}
    virtual void print_statistics() const override;

    virtual MSGSCollection get_msgs() const override;
    virtual void init_msgs(MSGSCollection goals) override;
};

}

#endif
