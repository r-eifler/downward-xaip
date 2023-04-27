#include "reachable_goal_subsets_max_heuristic.h"

#include "../../option_parser.h"
#include "../../plugin.h"
#include "../../evaluation_context.h"

#include "../../utils/logging.h"

#include "../goal_subsets/goal_subset.h"

#include <cassert>
#include <vector>

using namespace std;
using namespace max_heuristic;
using namespace goalsubset;

// construction and destruction
ReachableGoalSubsetMaxHeuristic::ReachableGoalSubsetMaxHeuristic(const Options &opts)
    : HSPMaxHeuristic(opts), 
      cost_bound(opts.get<int>("cost_bound")),
      is_cost_bounded(cost_bound > 0){

    current_msgs = MSGSCollection();
    current_msgs.initialize(task);

    if (log.is_at_least_normal()) {
        log << "Initializing reachable goal subsets max heuristic..." << endl;
    }

    overall_timer.reset();
}

EvaluationResult ReachableGoalSubsetMaxHeuristic::compute_result(EvaluationContext& context){
    State state = context.get_state();

    overall_timer.resume();
    vector<int> costs = get_heuristic_values(state, current_msgs.get_goal_facts());
    overall_timer.stop();

    //TODO handle instances with no explecit cost bound
    int remaining_cost = is_cost_bounded ? cost_bound - context.get_g_value() : INT_MAX;

    // cout << "Remaining cost: " << remaining_cost << endl;

    bool should_be_pruned = current_msgs.prune(state, costs, remaining_cost);

    EvaluationResult result;
    result.set_count_evaluation(true);
    if(should_be_pruned){
        num_pruned_states++;
        if(is_cost_bounded){
            // cout << cost_bound << " - " <<  context.get_g_value() << " = " << cost_bound - context.get_g_value() << endl;
            // cout << "prune"  << endl;
            result.set_evaluator_value(cost_bound - context.get_g_value()); //TODO why?
        }
        else 
            result.set_evaluator_value(EvaluationResult::INFTY);
        return result;
    }

    /*
        TODO is this possible: if state is not pruned then use the estimation of the most costly 
        reachable goal fact
    */
    // int total_cost = 0;
    // for (PropID goal_id : goal_propositions) {
    //     const Proposition *goal = get_proposition(goal_id);
    //     int goal_cost = goal->cost;
    //     total_cost = max(total_cost, goal_cost);
    // }

    result.set_evaluator_value(0);
    return result;
}

void ReachableGoalSubsetMaxHeuristic::print_statistics() const {
    cout << "num pruned states: " << num_pruned_states << endl;
    cout << "hmax computation time: " << overall_timer << endl;
    current_msgs.print();
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis("Reachable goal subset Max heuristic", "");

    Heuristic::add_options_to_parser(parser);

    parser.add_option<int>(
        "cost_bound",
        "overall costbound",
        "-1"
    );

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<ReachableGoalSubsetMaxHeuristic>(opts);
}


static Plugin<Evaluator> _plugin("rgsshmax", _parse);

