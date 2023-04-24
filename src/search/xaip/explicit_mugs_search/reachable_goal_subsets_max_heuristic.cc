#include "reachable_goal_subsets_max_heuristic.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/logging.h"

#include "../goal_subsets/goal_subset.h"

#include <cassert>
#include <vector>

using namespace std;
using namespace max_heuristic;
using namespace goalsubset;

// construction and destruction
ReachableGoalSubsetMaxHeuristic::ReachableGoalSubsetMaxHeuristic(const Options &opts)
    : HSPMaxHeuristic(opts), 
      cost_bound(opts.get<int>("cost_bound")){

    current_msgs = MSGSCollection();
    current_msgs.initialize(task);

    if (log.is_at_least_normal()) {
        log << "Initializing reachable goal subsets max heuristic..." << endl;
    }
}

int ReachableGoalSubsetMaxHeuristic::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);

    vector<int> costs = get_heuristic_values(state, current_msgs.get_goal_facts());

    //TODO handle instances with no explecit cost bound
    bool should_be_pruned = current_msgs.prune(state, costs, INT_MAX);

    if(should_be_pruned){
        return DEAD_END;
    }

    // if state is not pruned then use the estimation of the most costly 
    // reachable goal fact
    int total_cost = 0;
    for (PropID goal_id : goal_propositions) {
        const Proposition *goal = get_proposition(goal_id);
        int goal_cost = goal->cost;
        total_cost = max(total_cost, goal_cost);
    }
    return total_cost;
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis("Reachable goal subset Max heuristic", "");

    Heuristic::add_options_to_parser(parser);

    parser.add_option<bool>(
    "cost_bound",
    "overall costbound",
    "MAX_INT");

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<ReachableGoalSubsetMaxHeuristic>(opts);
}


static Plugin<Evaluator> _plugin("rgsshmax", _parse);

