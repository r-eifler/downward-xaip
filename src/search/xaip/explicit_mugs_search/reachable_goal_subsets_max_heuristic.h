#ifndef REACHABLE_GOAL_SUBSET_MAX_HEURISTIC_H
#define REACHABLE_GOAL_SUBSET_MAX_HEURISTIC_H

#include "../../heuristics/max_heuristic.h"
#include "../../heuristic.h"

#include "../algorithms/priority_queues.h"
#include "../../utils/timer.h"

#include "msgs_collection.h"

#include <cassert>
#include <boost/dynamic_bitset.hpp>


class ReachableGoalSubsetMaxHeuristic : public max_heuristic::HSPMaxHeuristic {

private:
    int cost_bound;
    bool is_cost_bounded;
    
    int num_pruned_states = 0;
    MSGSCollection current_msgs;

    utils::Timer overall_timer;

protected:

    virtual int compute_heuristic(const State&) override { return 0; }

public:
    virtual EvaluationResult compute_result(EvaluationContext& context) override;
    explicit ReachableGoalSubsetMaxHeuristic(const options::Options &opts);

    virtual void print_statistics() const override;
};


#endif
