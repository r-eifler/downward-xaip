#include "new_goal_subset_heuristic.h"

#include "msgs_evaluation_context.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace goalsubset;

namespace new_goal_subset_heuristic {
NewGoalSubsetHeuristic::NewGoalSubsetHeuristic(const Options &opts)
    : Heuristic(opts), heuristic_cache(HGEntry(NO_VALUE, 0, true)),
    h(opts.get<shared_ptr<Evaluator>>("h", nullptr)){

    log << "--> new goal subset heuristic" << endl;
    if(initialized){
        return;
    }
    initialized = true;

    goals_heuristic = dynamic_pointer_cast<Heuristic>(h);


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


    log << "initialize heuristic: new goal subset heuristic" << endl;
}

GoalSubsets NewGoalSubsetHeuristic::get_msgs() const {
    return GoalSubsets();
}

void NewGoalSubsetHeuristic::init_msgs(MSGSCollection) {
}

int NewGoalSubsetHeuristic::min(int x, int y){
    if(x == -1){
        return y;
    }
    if(y == -1){
        return x;
    }
    if(x < y){
        return x;
    }
    else{
        return y;
    }
}


int NewGoalSubsetHeuristic::max(int x, int y){
    if(x == -1){
        return x;
    }
    if(y == -1){
        return y;
    }
    if(x > y){
        return x;
    }
    else{
        return y;
    }
}


EvaluationResult NewGoalSubsetHeuristic::compute_result(EvaluationContext &eval_context) {
    EvaluationResult result;

    MSGSEvaluationContext* msgs_eval_context = dynamic_cast<MSGSEvaluationContext*>(&eval_context);

    const State &state = msgs_eval_context->get_state();
    const int g = msgs_eval_context-> get_g_value();
    MSGSCollection* current_msgs = msgs_eval_context->get_msgs_collection();
    int remaining_cost = msgs_eval_context->get_cost_bound() - msgs_eval_context->get_g_value();
    // cout << "Size current MUGS: " << current_msgs->size() << endl;

    int heuristic = NO_VALUE;

    if (cache_evaluator_values &&
        heuristic_cache[state].h != NO_VALUE && 
        heuristic_cache[state].g == g &&
        !heuristic_cache[state].dirty) {
        heuristic = heuristic_cache[state].h;
        result.set_count_evaluation(false);
    } else {
        heuristic = compute_heuristic(state, current_msgs, remaining_cost);
        if (cache_evaluator_values) {
            heuristic_cache[state] = HGEntry(heuristic, g, false);
        }
        result.set_count_evaluation(true);
    }

    assert(heuristic == DEAD_END || heuristic >= 0);

    if (heuristic == DEAD_END) {
        heuristic = EvaluationResult::INFTY;
    }

    result.set_evaluator_value(heuristic);
    return result;
}

int NewGoalSubsetHeuristic::compute_heuristic(const State &state, MSGSCollection* current_msgs, int remaining_cost){

    // cout << "new goal subset heuristic: " << state.get_id() << " = ";

    vector<int> costs = goals_heuristic->get_heuristic_values(state, all_goal_list);
    // costs contains the heuristic estimate of all goal fact in the order in
    // all_goal_list (first hard goals, then soft goals)

    //check all hard goals reachable
    int res = current_msgs->prune(state, costs, remaining_cost);
    // cout << res << endl;
    return res;
}



int NewGoalSubsetHeuristic::compute_heuristic(const State &state){

    // cout << "new goal subset heuristic: -------------------------------" << endl;

    vector<int> costs = goals_heuristic->get_heuristic_values(state, all_goal_list);
    int res = 0;
    for(size_t i = 0; i < costs.size(); i++){
        res = max(res,costs[i]);
    }
    return res;
}


void NewGoalSubsetHeuristic::print_statistics() const {
    cout << "##############################################################" << endl;
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
