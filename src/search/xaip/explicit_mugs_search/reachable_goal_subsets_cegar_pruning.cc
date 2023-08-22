#include "reachable_goal_subsets_cegar_pruning.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace goalsubset;

namespace reachable_goal_subsets_cegar_pruning {
ReachableGoalSubsetsCegarPruning::ReachableGoalSubsetsCegarPruning(const Options &opts)
    : PruningMethod(opts),
    h(opts.get<shared_ptr<Evaluator>>("h", nullptr)){

    log << "--> reachable goal subset pruning" << endl;
    
}

void ReachableGoalSubsetsCegarPruning::initialize(const shared_ptr<AbstractTask> &task) {
    if(initialized){
        return;
    }
    initialized = true;

    PruningMethod::initialize(task);

    cegar_heuristic = static_pointer_cast<cegar::AdditiveCartesianHeuristic>(h);

    current_msgs = MSGSCollection();
    current_msgs.initialize(task);

    log << "initialize pruning method: reachable goal subset pruning" << endl;
}

MSGSCollection ReachableGoalSubsetsCegarPruning::get_msgs() const {
    return current_msgs;
}

void ReachableGoalSubsetsCegarPruning::init_msgs(MSGSCollection subsets) {
    cout << "init msgs: size: " << subsets.size() << endl;
    current_msgs = subsets;
}


bool ReachableGoalSubsetsCegarPruning::prune(const State &state, int remaining_cost){

    // cout << "---------------------------------------------------------------" << endl;
    // cout << "prune state remaining cost: "  << remaining_cost << endl;
    // cout << "-------------------" << endl;
    // for(size_t i = 0; i < state.size(); i++)
    //     cout << state[i].get_variable().get_id() << " = " << state[i].get_value()  << "    -->  " << state[i].get_name() << endl;
    // cout << "-------------------" << endl;

    vector<int> costs = cegar_heuristic->get_heuristic_values(state, current_msgs.get_goal_facts());

    return current_msgs.prune(state, costs, remaining_cost);
}

void ReachableGoalSubsetsCegarPruning::print_statistics() const {
    current_msgs.print();
}


static shared_ptr<PruningMethod> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "reachable goal subsets",
        "States are pruned if no subset of goals already seen is reachable");
    add_pruning_options_to_parser(parser);

    parser.add_option<shared_ptr<Evaluator>>(
        "h",
        "add max heuristic");

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<ReachableGoalSubsetsCegarPruning>(opts);
}

static Plugin<PruningMethod> _plugin("rgsspcegar", _parse);
}
