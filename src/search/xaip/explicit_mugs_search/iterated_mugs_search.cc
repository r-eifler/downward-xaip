#include "iterated_mugs_search.h"

#include "../../search_engines/eager_search.h"
#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/logging.h"

#include <iostream>

using namespace std;

namespace iterated_mugs_search {
IteratedMUGSSearch::IteratedMUGSSearch(const Options &opts, options::Registry &registry,
                               const options::Predefinitions &predefinitions)
    : SearchEngine(opts),
      engine_configs(opts.get_list<ParseTree>("engine_configs")),
      registry(registry),
      predefinitions(predefinitions),
      phase(0),
      iterated_found_solution(false),
      pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")) {

}

void IteratedMUGSSearch::initialize() {
    pruning_method->initialize(task);
}

shared_ptr<SearchEngine> IteratedMUGSSearch::get_search_engine(int engine_configs_index) {

    OptionParser parser(engine_configs[engine_configs_index], registry, predefinitions, false);
    shared_ptr<SearchEngine> engine(parser.start_parsing<shared_ptr<SearchEngine>>());

    ostringstream stream;
    kptree::print_tree_bracketed(engine_configs[engine_configs_index], stream);
    log << "Starting search: " << stream.str() << endl;

    return engine;
}

shared_ptr<SearchEngine> IteratedMUGSSearch::create_current_phase() {
    int num_phases = engine_configs.size();
    if (phase >= num_phases) {
        return nullptr;
    }

    return get_search_engine(phase);
}

SearchStatus IteratedMUGSSearch::step() {
    cout << "---------------------------------------------------------" << endl;
    cout << "---------------------------------------------------------" << endl;
    shared_ptr<SearchEngine> current_search = create_current_phase();

    static_pointer_cast<eager_search::EagerSearch>(current_search)->set_pruning_method(this->pruning_method);

    if (!current_search) {
        return found_solution() ? SOLVED : FAILED;
    }
    ++phase;

    current_search->search();

    Plan found_plan;
    iterated_found_solution = current_search->found_solution();
    if (iterated_found_solution) {
        found_plan = current_search->get_plan();
        plan_manager.save_plan(found_plan, task_proxy, true);
        set_plan(found_plan);
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

SearchStatus IteratedMUGSSearch::step_return_value() {
    if(iterated_found_solution){
        return SOLVED;
    }
    int num_phases = engine_configs.size();
    return phase >= num_phases ? FAILED : IN_PROGRESS;  
}

void IteratedMUGSSearch::print_statistics() const {
    log << "Cumulative statistics:" << endl;
    statistics.print_detailed_statistics();
}

void IteratedMUGSSearch::save_plan_if_necessary() {
    // We don't need to save here, as we automatically save after
    // each successful search iteration.
}

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
    parser.document_synopsis("Iterated search", "");
    parser.document_note(
        "Note 1",
        "We don't cache heuristic values between search iterations at"
        " the moment. If you perform a LAMA-style iterative search,"
        " heuristic values will be computed multiple times.");
    parser.document_note(
        "Note 2",
        "The configuration\n```\n"
        "--search \"iterated([lazy_wastar(merge_and_shrink(),w=10), "
        "lazy_wastar(merge_and_shrink(),w=5), lazy_wastar(merge_and_shrink(),w=3), "
        "lazy_wastar(merge_and_shrink(),w=2), lazy_wastar(merge_and_shrink(),w=1)])\"\n"
        "```\nwould perform the preprocessing phase of the merge and shrink heuristic "
        "5 times (once before each iteration).\n\n"
        "To avoid this, use heuristic predefinition, which avoids duplicate "
        "preprocessing, as follows:\n```\n"
        "--evaluator \"h=merge_and_shrink()\" --search "
        "\"iterated([lazy_wastar(h,w=10), lazy_wastar(h,w=5), lazy_wastar(h,w=3), "
        "lazy_wastar(h,w=2), lazy_wastar(h,w=1)])\"\n"
        "```");
    parser.document_note(
        "Note 3",
        "If you reuse the same landmark count heuristic "
        "(using heuristic predefinition) between iterations, "
        "the path data (that is, landmark status for each visited state) "
        "will be saved between iterations.");
    parser.add_list_option<ParseTree>("engine_configs",
                                      "list of search engines for each phase");
    parser.add_option<shared_ptr<PruningMethod>>(
        "pruning",
        "Pruning methods can prune or reorder the set of applicable operators in "
        "each state and thereby influence the number and order of successor states "
        "that are considered.",
        "null()");
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
        return make_shared<IteratedMUGSSearch>(opts, parser.get_registry(),
                                           parser.get_predefinitions());
    }
}

static Plugin<SearchEngine> _plugin("iterated_mugs", _parse);
}
