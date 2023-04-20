#include "dualization.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/logging.h"

#include "goal_subset_space.h"
#include "../../tasks/modified_goals_task.h"
#include "../tasks/root_task.h"
#include "../heuristic.h"

#include <iostream>

using namespace std;
using namespace goalsubset;

namespace dualization_search {
DualizationSearch::DualizationSearch(const Options &opts, options::Registry &registry,
                               const options::Predefinitions &predefinitions)
    : SearchEngine(opts),
      engine_config(opts.get<ParseTree>("engine_config")),
      registry(registry),
      predefinitions(predefinitions),
      all_soft_goals(opts.get<bool>("all_soft_goals")),
      weakening(opts.get<bool>("weakening")){
    
    std::vector<shared_ptr<Evaluator>> evaluators = opts.get_list<shared_ptr<Evaluator>>("heu");
    for (shared_ptr<Evaluator> eval : evaluators) {
        Evaluator* e_pointer = eval.get();
        heuristic.push_back(dynamic_cast<Heuristic*>(e_pointer));
        assert(heuristic.back() != nullptr);
    }

    TaskProxy task_proxy = TaskProxy(*tasks::g_root_task.get());

    if(all_soft_goals){
        for(uint i = 0; i < task_proxy.get_goals().size(); i++){
            soft_goal_list.push_back(task_proxy.get_goals()[i].get_pair());
        }
    }
    else{
        for(uint i = 0; i < task_proxy.get_soft_goals().size(); i++){
            soft_goal_list.push_back(task_proxy.get_soft_goals()[i].get_pair());
        }
        for(uint i = 0; i < task_proxy.get_hard_goals().size(); i++){
            hard_goal_list.push_back(task_proxy.get_hard_goals()[i].get_pair());
        }
    }
    
    //sort goal facts according to there variable id
    std::sort(soft_goal_list.begin(), soft_goal_list.end());

     for(uint i = 0; i < soft_goal_list.size(); i++){
        FactPair gp = soft_goal_list[i];
        soft_goal_fact_names.push_back(task_proxy.get_variables()[gp.var].get_fact(gp.value).get_name());
    }

    //init with empty set;
    satisfiable_set = GoalSubset(soft_goal_list.size());
    maximal_satisfiable_set = GoalSubset(soft_goal_list.size());
    
    cout << "INIT GOAL DUALIZATION SEARCH" << endl;
}

vector<FactPair> DualizationSearch::get_goals(GoalSubset set) const{
    vector<FactPair> soft_goals;

    for (uint i = 0; i < soft_goal_list.size(); i++) {
        if (set.contains(i)) {
            soft_goals.push_back(soft_goal_list[i]);
        }
    }

    vector<FactPair> current_goals;
    current_goals.insert( current_goals.end(), hard_goal_list.begin(), hard_goal_list.end() );
    current_goals.insert( current_goals.end(), soft_goals.begin(), soft_goals.end() );

    return current_goals;
}

bool DualizationSearch::call_search_engine(goalsubset::GoalSubset set) {

    if(planner_called_solvable.contains(set)){
        return true;
    }

    if(planner_called_unsolvable.contains(set)){
        return false;
    }

    num_planner_calls++;
    tasks::g_root_task = make_shared<extra_tasks::ModifiedGoalsTask>(task, get_goals(set)); 

    for (Heuristic* h : heuristic) {
        h->set_abstract_task(tasks::g_root_task);
    }

    OptionParser parser(engine_config, registry, predefinitions, false);
    shared_ptr<SearchEngine> engine(parser.start_parsing<shared_ptr<SearchEngine>>());

    engine->search();

    if(engine->found_solution()){
        planner_called_solvable.add(set);
    }
    else{
        planner_called_unsolvable.add(set);
    }

    return engine->found_solution();
}

goalsubset::GoalSubset DualizationSearch::grow(goalsubset::GoalSubset set){
    // cout << "---- grow -----" << endl;
    GoalSubset grow_set = set;
    for(long unsigned int i = 0; i < grow_set.size(); i++){
        if(!grow_set.contains(i)){
            GoalSubset test = grow_set.set_union(GoalSubset(i, grow_set.size()));
            // test.print();
            if(call_search_engine(test)){
                // cout << "solvable" << endl;
                grow_set = test;
            }
            // else{
            //     cout << "not solvable" << endl;
            // }
        }
    }
    return grow_set;
}

SearchStatus DualizationSearch::step() {

    // cout << "----------------------------------------------------------------------------" << endl;
    // cout << "---- satisfiable set -----" << endl;
    // satisfiable_set.print();
    // cout << "------------------------------------" << endl;
    // cout << "---- maximal_satisfiable_set -----" << endl;
    // maximal_satisfiable_set.print();
    // cout << "------------------------------------" << endl;
    // cout << "---- current_MUGS -----" << endl;
    // current_MUGS.print_subsets();
    // cout << "------------------------------------" << endl;
    // cout << "---- comp_current_MSGS -----" << endl;
    // comp_current_MSGS.print_subsets();
    // cout << "------------------------------------" << endl;
    // cout << "---- candidates_MUGS -----" << endl;
    // candidates_MUGS.print_subsets();
    // cout << "------------------------------------" << endl;
    
    satisfiable_set = grow(satisfiable_set);
    // cout << "---- next <- grown -----" << endl;
    // satisfiable_set.print();
    // cout << "------------------------------------" << endl;

    comp_current_MSGS.add(satisfiable_set.complement());

    candidates_MUGS = comp_current_MSGS.minimal_hitting_sets();

    satisfiable_set = GoalSubset(); //TODO an max num goals;

    for(GoalSubset test : candidates_MUGS.minus(current_MUGS)){
        if(call_search_engine(test)){
            satisfiable_set = test;
            break;
        }
        else{
            current_MUGS.add(test);
        }
    }

    // cout << "----------------------------------------------------------------------------" << endl;
    return !satisfiable_set.is_empty() ? IN_PROGRESS : FINISHED;
}


void DualizationSearch::print_statistics() const {
    // log << "Cumulative statistics:" << endl;
    // statistics.print_detailed_statistics();

    cout << "*********************************"  << endl;
    cout << "#planner calls: " << num_planner_calls << endl;
    // planner_called_solvable.print_subsets();
    // cout << "--" << endl;
    // planner_called_unsolvable.print_subsets();
    cout << "*********************************"  << endl;
    cout << "#hard goals: " << hard_goal_list.size() << endl;
    TaskProxy taskproxy = TaskProxy(*tasks::g_root_task.get());
    for(FactPair g : hard_goal_list){
        cout << "\t" << taskproxy.get_variables()[g.var].get_fact(g.value).get_name() << endl;
    }
    cout << "#soft goals: " << soft_goal_list.size() << endl;
    for(FactPair g : soft_goal_list){
        cout << "\t" << taskproxy.get_variables()[g.var].get_fact(g.value).get_name() << endl;
    }
    cout << "*********************************"  << endl;
    cout << "#MUGS: " << current_MUGS.size() << endl;
    cout << "*********************************"  << endl;
    current_MUGS.print_subsets();
    cout << "*********************************"  << endl;
    current_MUGS.print(soft_goal_fact_names);
    cout << "*********************************"  << endl;
}

void DualizationSearch::save_plan_if_necessary() {
    // We don't need to save here, as we automatically save after
    // each successful search iteration.
}

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {

    parser.add_option<ParseTree>("engine_config",
                                "search engine for each search");
    parser.add_option<bool>("all_soft_goals",
                            "treat all goals as soft goals",
                            "false");
    parser.add_list_option<shared_ptr<Evaluator>>("heu", "reference to heuristic to update abstract task");

    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();


    if (parser.help_mode()) {
        return nullptr;
    } else if (parser.dry_run()) {
        //check if the supplied search engines can be parsed
        OptionParser test_parser(opts.get<ParseTree>("engine_config"), parser.get_registry(),
                                     parser.get_predefinitions(), true);
        test_parser.start_parsing<shared_ptr<SearchEngine>>();
        
        return nullptr;
    } else {
        opts.set("weakening", false);
        return make_shared<DualizationSearch>(opts, parser.get_registry(), parser.get_predefinitions());
    }
}

static Plugin<SearchEngine> _plugin("dual", _parse);

}
