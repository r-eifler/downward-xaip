#include "initial_state_heuristic.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../algorithms/ordered_set.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/pref_evaluator.h"
#include "../open_lists/best_first_open_list.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../task_utils/successor_generator.h"
#include "../utils/logging.h"
#include "../utils/system.h"

using namespace std;
using utils::ExitCode;

namespace initial_state_heuristic_search {


InitialStateHeurisicSearch::InitialStateHeurisicSearch(
    const Options &opts)
    : SearchEngine(opts),
      evaluator(opts.get<shared_ptr<Evaluator>>("h")) {

}

InitialStateHeurisicSearch::~InitialStateHeurisicSearch() {
}

void InitialStateHeurisicSearch::initialize() {
    State initial_state = state_registry.get_initial_state();
    EvaluationContext eval_context(initial_state, 0, true, &statistics);

    eval_context.is_evaluator_value_infinite(evaluator.get());
    print_initial_evaluator_values(eval_context);
}



SearchStatus InitialStateHeurisicSearch::step() {
    return SOLVED;
}



void InitialStateHeurisicSearch::print_statistics() const {
  
}

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
    parser.document_synopsis("Initial State Heuristic Value", "");
    parser.add_option<shared_ptr<Evaluator>>("h", "heuristic");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<InitialStateHeurisicSearch>(opts);
}

static Plugin<SearchEngine> _plugin("initial_state_heuristic", _parse);
}
