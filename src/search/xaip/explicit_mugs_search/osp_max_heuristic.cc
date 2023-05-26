#include "osp_max_heuristic.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../utils/logging.h"

#include <cassert>
#include <vector>

using namespace std;
using namespace max_heuristic;


// construction and destruction
OSPMaxHeuristic::OSPMaxHeuristic(const Options &opts)
    : HSPMaxHeuristic(opts) {
    if (log.is_at_least_normal()) {
        log << "Initializing HSP max heuristic..." << endl;
    }
}


int OSPMaxHeuristic::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);

    setup_exploration_queue();
    setup_exploration_queue_state(state);
    relaxed_exploration();

    int max_cost = 0;
    for (PropID goal_id : goal_propositions) {
        const Proposition *goal = get_proposition(goal_id);
        int goal_cost = goal->cost;
        max_cost = max(max_cost, goal_cost);
    }
    // cout << total_cost << endl;
    return max_cost;
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis("Max heuristic", "");
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects", "supported");
    parser.document_language_support(
        "axioms",
        "supported (in the sense that the planner won't complain -- "
        "handling of axioms might be very stupid "
        "and even render the heuristic unsafe)");
    parser.document_property("admissible", "yes for tasks without axioms");
    parser.document_property("consistent", "yes for tasks without axioms");
    parser.document_property("safe", "yes for tasks without axioms");
    parser.document_property("preferred operators", "no");

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<OSPMaxHeuristic>(opts);
}


static Plugin<Evaluator> _plugin("osphmax", _parse);

