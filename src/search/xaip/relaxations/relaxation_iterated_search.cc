#include "relaxation_iterated_search.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "modified_init_task.h"
#include "../tasks/root_task.h"
#include "../heuristic.h"
#include "../goal_subsets/goal_subset.h"
#include "../goal_subsets/output_handler.h"

#include <iostream>

using namespace std;
using namespace goalsubset;

namespace relaxation_iterated_search {
RelaxationIteratedSearch::RelaxationIteratedSearch(const Options &opts, options::Registry &registry,
                               const options::Predefinitions &predefinitions)
    : SearchEngine(opts),
    engine_configs(opts.get_list<ParseTree>("engine_configs")),
    registry(registry),
    predefinitions(predefinitions),
    propagate_msgs(opts.get<bool>("propagate_msgs")){

    taskRelaxationTracker = new TaskRelaxationTracker(task);

    std::vector<shared_ptr<Evaluator>> evaluators = opts.get_list<shared_ptr<Evaluator>>("heu");
    for (shared_ptr<Evaluator> eval : evaluators) {
        Evaluator* e_pointer = eval.get();
        heuristic.push_back(dynamic_cast<Heuristic*>(e_pointer));
        assert(heuristic.back() != nullptr);
    }
}

shared_ptr<SearchEngine> RelaxationIteratedSearch::get_search_engine() {

    //update global root task with current relaxed task
    relaxedTask = taskRelaxationTracker->next_relaxed_task();
    tasks::g_root_task = make_shared<extra_tasks::ModifiedInitTask>(task, relaxedTask->get_init());

    cout << "*******************************************************" << endl;
    cout << "Iteration: " << relaxedTask->get_name() << endl;

    cout << "Init state: "  << endl;
    for (FactPair fp : relaxedTask->get_init())
        cout << fp.var << " = " << fp.value << endl;

    for (Heuristic* h : heuristic) {
        h->set_abstract_task(tasks::g_root_task);


        GoalSubsets init_msgs;
        if(propagate_msgs) {
            for (RelaxedTask *t: relaxedTask->get_lower_cover()) {
                GoalSubsets t_msgs = t->get_msgs();
                for(GoalSubset gs : t_msgs)
                    init_msgs.add(gs);
            }
        }
        h->init_msgs(init_msgs);
    }

    OptionParser parser(engine_configs[0], registry, predefinitions, false);
    shared_ptr<SearchEngine> engine(parser.start_parsing<shared_ptr<SearchEngine>>());

    cout << "Starting search: ";
    kptree::print_tree_bracketed(engine_configs[0], cout);
    cout << endl;

    return engine;
}

shared_ptr<SearchEngine> RelaxationIteratedSearch::create_phase() {

    // propagate solvable in case the current task was solved
    if (relaxedTask && relaxedTask->get_solvable()){
        cout << relaxedTask->get_name() << endl;
        relaxedTask->propagate_solvable(relaxedTask->get_msgs());
    }

    if (taskRelaxationTracker->has_next_relaxed_task()) {
        return get_search_engine();
    } else {
        return nullptr;
    }
}

SearchStatus RelaxationIteratedSearch::step() {
    shared_ptr<SearchEngine> current_search = create_phase();
    if (!current_search) {
        return found_solution() ? SOLVED : FAILED;
    }

    current_search->search();

    for (Heuristic* h : heuristic) {
        GoalSubsets msgs = h->get_msgs();
        relaxedTask->set_msgs(msgs);
        h->compute_mugs();
        relaxedTask->set_mugs(h->get_mugs());
    }

    if (current_search->found_solution()){
        cout << "solved Iteration: " << relaxedTask->get_name() << endl;
        relaxedTask->propagate_solvable(relaxedTask->get_msgs());
    }

    current_search->print_statistics();

    const SearchStatistics &current_stats = current_search->get_statistics();
    statistics.inc_expanded(current_stats.get_expanded());
    statistics.inc_evaluated_states(current_stats.get_evaluated_states());
    statistics.inc_evaluations(current_stats.get_evaluations());
    statistics.inc_generated(current_stats.get_generated());
    statistics.inc_generated_ops(current_stats.get_generated_ops());
    statistics.inc_reopened(current_stats.get_reopened());

    return step_return_value();
}

SearchStatus RelaxationIteratedSearch::step_return_value() {
    if(taskRelaxationTracker->has_next_relaxed_task()){
        return IN_PROGRESS;
    }

    return SOLVED;
}

void RelaxationIteratedSearch::print_statistics() const {
    cout << "Cumulative statistics:" << endl;
    statistics.print_detailed_statistics();

    // print MUGS to file
    int num_goal_facts = task_proxy.get_goals().size();
    std::vector<std::string> goal_fact_names;
    for(int i = 0; i < num_goal_facts; i++){
        FactProxy gp = task_proxy.get_goals()[i];
        int id = gp.get_variable().get_id();
        int value = gp.get_value();
        goal_fact_names.push_back(task_proxy.get_variables()[id].get_fact(value).get_name());
    }
    OutputHandler output_handler = OutputHandler(goal_fact_names, "relaxation_mugs.json", true);
    for(RelaxedTask* task: this->taskRelaxationTracker->get_relaxed_tasks()){
        output_handler.add_goal_subsets(task->get_name(), task->get_mugs());
    }
    output_handler.output();
}

void RelaxationIteratedSearch::save_plan_if_necessary() {
    // We don't need to save here, as we automatically save after
    // each successful search iteration.
}

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
    parser.document_synopsis("Relaxation Iterated search", "");
    parser.add_option<bool>("propagate_msgs",
                            "propagate the maximally solvable goal subsets",
                            "true");
    parser.add_list_option<shared_ptr<Evaluator>>("heu", "reference to heuristic to update abstract task");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    opts.verify_list_non_empty<ParseTree>("engine_configs");

    if (parser.help_mode()) {
        return nullptr;
    } else if (parser.dry_run()) {
        //check if the supplied search engines can be parsed
        for (const ParseTree &config : opts.get_list<ParseTree>("engine_configs")) {
            OptionParser test_parser(config, parser.get_registry(),
                                     parser.get_predefinitions(), true);
            test_parser.start_parsing<shared_ptr<SearchEngine>>();
        }
        return nullptr;
    } else {
        return make_shared<RelaxationIteratedSearch>(opts, parser.get_registry(),
                                           parser.get_predefinitions());
    }
}

static Plugin<SearchEngine> _plugin("relaxation_iterated", _parse);
}
