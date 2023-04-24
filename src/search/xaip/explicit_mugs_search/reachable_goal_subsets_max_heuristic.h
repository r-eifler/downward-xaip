#ifndef REACHABLE_GOAL_SUBSET_MAX_HEURISTIC_H
#define REACHABLE_GOAL_SUBSET_MAX_HEURISTIC_H

#include "../heuristics/max_heuristic.h"

#include "../algorithms/priority_queues.h"

#include "msgs_collection.h"

#include <cassert>
#include <boost/dynamic_bitset.hpp>


class ReachableGoalSubsetMaxHeuristic : public max_heuristic::HSPMaxHeuristic {

private:
    int cost_bound;
    
    MSGSCollection current_msgs;

protected:

    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    explicit ReachableGoalSubsetMaxHeuristic(const options::Options &opts);
};


#endif
