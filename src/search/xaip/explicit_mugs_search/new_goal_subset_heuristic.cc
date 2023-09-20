#include "new_goal_subset_heuristic.h"

#include <algorithm>

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace goalsubset;

namespace new_goal_subset_heuristic {
NewGoalSubsetHeuristic::NewGoalSubsetHeuristic(const Options &opts)
    : Heuristic(opts),
    h(opts.get<shared_ptr<Evaluator>>("h", nullptr)){

    log << "--> new goal subset heuristic with cartesian abstractions" << endl;
    if(initialized){
        return;
    }
    initialized = true;

    cegar_heuristic = static_pointer_cast<cegar::AdditiveCartesianHeuristic>(h);


    soft_goal_list = vector<FactPair>();
    hard_goal_list = vector<FactPair>();

    if(task_proxy.get_hard_goals().size() == 0){
        cout << "--> all goals are soft goals" << endl;
        for(uint i = 0; i < task_proxy.get_goals().size(); i++){
            soft_goal_list.push_back(task_proxy.get_goals()[i].get_pair());
            all_goal_list.push_back(task_proxy.get_goals()[i].get_pair());
        }
    }
    else{
        for(uint i = 0; i < task_proxy.get_hard_goals().size(); i++){
            hard_goal_list.push_back(task_proxy.get_hard_goals()[i].get_pair());
            all_goal_list.push_back(task_proxy.get_hard_goals()[i].get_pair());
        }
        for(uint i = 0; i < task_proxy.get_soft_goals().size(); i++){
            soft_goal_list.push_back(task_proxy.get_soft_goals()[i].get_pair());
            all_goal_list.push_back(task_proxy.get_soft_goals()[i].get_pair());
        }
    }

    // cout << "#hard goals: " << hard_goal_list.size() << endl;

    //sort goal facts according to there variable id
    std::sort(hard_goal_list.begin(), hard_goal_list.end());

    // cout << "#soft goals: " << soft_goal_list.size() << endl;

    //sort goal facts according to there variable id
    std::sort(soft_goal_list.begin(), soft_goal_list.end());


    current_msgs = MSGSCollection();
    current_msgs.initialize(task);

    log << "initialize heuristic: new goal subset heuristic with cartesian abstractions" << endl;
}

GoalSubsets NewGoalSubsetHeuristic::get_msgs() const {
    return current_msgs;
}

void NewGoalSubsetHeuristic::init_msgs(MSGSCollection subsets) {
    cout << "init msgs: size: " << subsets.size() << endl;
    current_msgs = subsets;
}


int NewGoalSubsetHeuristic::compute_heuristic(const State &state){

    //TODO handle infiniy (-1) correctely

    // cout << "---------------------------------------------------------------" << endl;
    // cout << "prune state remaining cost: "  << remaining_cost << endl;
    // cout << "-------------------" << endl;
    // for(size_t i = 0; i < state.size(); i++)
    //     cout << state[i].get_variable().get_id() << " = " << state[i].get_value()  << "    -->  " << state[i].get_name() << endl;
    // cout << "-------------------" << endl;

    current_msgs.track(state);

    vector<int> costs = cegar_heuristic->get_heuristic_values(state, all_goal_list);
    // costs contains the heuristic estimate of all goal fact in the order in
    // all_goal_list (first hard goals, then soft goals)

    //check all hard goals reachable
    int max_hard_goal = 0;
    GoalSubset reachable_goals = GoalSubset(all_goal_list.size());
    for(size_t i = 0; i < hard_goal_list.size(); i++){
        if(costs[i] == -1){
            return - 1;
        }
        max_hard_goal = max(costs[i], max_hard_goal);
    }

    int max_soft_goal = 0;
    for(GoalSubset gs : current_msgs){

        //max unsat
        int max_unsat = 0;
        for(size_t i = 0; i < soft_goal_list.size(); i++){
            FactPair g = soft_goal_list[i];
            if(gs.contains(i) && state[g.var].get_value() != g.value){
                max_unsat = max(max_unsat, costs[hard_goal_list.size() + i]);
            }
        }

        //min to sat
        int min_to_sat = 2147483647; //max int value
        for(size_t i = 0; i < soft_goal_list.size(); i++){
            if(! gs.contains(i)){
                min_to_sat =min(min_to_sat, costs[hard_goal_list.size() + i]);
            }
        }

        max_soft_goal = max(max(max_soft_goal, max_unsat), min_to_sat);

    }

    return max(max_hard_goal, max_soft_goal);
}


void NewGoalSubsetHeuristic::print_statistics() const {
    cout << "##############################################################" << endl;
    current_msgs.print();
}


static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "reachable goal subsets",
        "States are pruned if no subset of goals already seen is reachable");

    parser.add_option<shared_ptr<Evaluator>>(
        "h",
        "add max heuristic");

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<NewGoalSubsetHeuristic>(opts);
}

static Plugin<Evaluator> _plugin("ngs", _parse);
}
