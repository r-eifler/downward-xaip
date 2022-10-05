#include "goal_subset_search.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/logging.h"

#include "goal_subset_space.h"
#include "../../tasks/modified_goals_task.h"
#include "../tasks/root_task.h"
#include "../heuristic.h"

#include <iostream>

using namespace std;
using namespace goalsubsetspace;

namespace goal_subset_search {
GoalSubsetSearch::GoalSubsetSearch(const Options &opts, options::Registry &registry,
                               const options::Predefinitions &predefinitions)
    : SearchEngine(opts),
      engine_config(opts.get<ParseTree>("engine_config")),
      registry(registry),
      predefinitions(predefinitions),
      all_soft_goals(opts.get<bool>("all_soft_goals")),
      weakening(opts.get<bool>("weakening")){

        meta_search_space = new GoalSubsetSpace(task_proxy.get_goals(), all_soft_goals, weakening);
    
        std::vector<Evaluator*> evaluators = opts.get_list<Evaluator*>("heu");
        for (Evaluator* eval : evaluators) {
            heuristic.push_back(dynamic_cast<Heuristic*>(eval));
            assert(heuristic.back() != nullptr);
        }
}

shared_ptr<SearchEngine> GoalSubsetSearch::get_next_search_engine() {

    meta_search_space->next_node();
    tasks::g_root_task = make_shared<extra_tasks::ModifiedGoalsTask>(task, meta_search_space->get_next_goals()); 

    for (Heuristic* h : heuristic) {
        h->set_abstract_task(tasks::g_root_task);
    }

    OptionParser parser(engine_config, registry, predefinitions, false);
    shared_ptr<SearchEngine> engine(parser.start_parsing<shared_ptr<SearchEngine>>());

    // ostringstream stream;
    // kptree::print_tree_bracketed(engine_configs[engine_configs_index], stream);
    // log << "Starting search: " << stream.str() << endl;

    return engine;
}

SearchStatus GoalSubsetSearch::step() {
    shared_ptr<SearchEngine> search_engine = get_next_search_engine();

    //TODO add additional search status
    if (!search_engine) {
        return FINISHED;
    }

    search_engine->search();

    search_engine->print_statistics();

    meta_search_space->current_goals_solved(search_engine->found_solution());

    meta_search_space->expand();

    return meta_search_space->continue_search() ? IN_PROGRESS : FINISHED;
}


void GoalSubsetSearch::print_statistics() const {
    log << "Cumulative statistics:" << endl;
    statistics.print_detailed_statistics();
}

void GoalSubsetSearch::save_plan_if_necessary() {
    // We don't need to save here, as we automatically save after
    // each successful search iteration.
}

}
