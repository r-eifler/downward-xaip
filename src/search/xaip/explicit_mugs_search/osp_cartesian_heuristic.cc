#include "osp_cartesian_heuristic.h"

#include "../../cegar/cartesian_heuristic_function.h"
#include "../../cegar/cost_saturation.h"
#include "../../cegar/types.h"
#include "../../cegar/utils.h"

#include "../../option_parser.h"
#include "../../plugin.h"

#include "../../utils/logging.h"
#include "../../utils/markup.h"
#include "../../utils/rng.h"
#include "../../utils/rng_options.h"

#include <cassert>

using namespace std;
using namespace cegar;

namespace osp_cegar {

OSPCartesianHeuristic::OSPCartesianHeuristic(
    const options::Options &opts)
    : AdditiveCartesianHeuristic(opts) {
}

int OSPCartesianHeuristic::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);
    int max_cost = 0;
    for (const CartesianHeuristicFunction &function : heuristic_functions) {
        int value = function.get_value(state);
        assert(value >= 0);
        max_cost = max(max_cost, value);
    }
    assert(max_cost >= 0);
    return max_cost;
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Additive CEGAR heuristic",
        "See the paper introducing Counterexample-guided Abstraction "
        "Refinement (CEGAR) for classical planning:" +
        utils::format_conference_reference(
            {"Jendrik Seipp", "Malte Helmert"},
            "Counterexample-guided Cartesian Abstraction Refinement",
            "https://ai.dmi.unibas.ch/papers/seipp-helmert-icaps2013.pdf",
            "Proceedings of the 23rd International Conference on Automated "
            "Planning and Scheduling (ICAPS 2013)",
            "347-351",
            "AAAI Press",
            "2013") +
        "and the paper showing how to make the abstractions additive:" +
        utils::format_conference_reference(
            {"Jendrik Seipp", "Malte Helmert"},
            "Diverse and Additive Cartesian Abstraction Heuristics",
            "https://ai.dmi.unibas.ch/papers/seipp-helmert-icaps2014.pdf",
            "Proceedings of the 24th International Conference on "
            "Automated Planning and Scheduling (ICAPS 2014)",
            "289-297",
            "AAAI Press",
            "2014") +
        "For more details on Cartesian CEGAR and saturated cost partitioning, "
        "see the journal paper" +
        utils::format_journal_reference(
            {"Jendrik Seipp", "Malte Helmert"},
            "Counterexample-Guided Cartesian Abstraction Refinement for "
            "Classical Planning",
            "https://ai.dmi.unibas.ch/papers/seipp-helmert-jair2018.pdf",
            "Journal of Artificial Intelligence Research",
            "62",
            "535-577",
            "2018"));
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects", "not supported");
    parser.document_language_support("axioms", "not supported");
    parser.document_property("admissible", "yes");
    parser.document_property("consistent", "yes");
    parser.document_property("safe", "yes");
    parser.document_property("preferred operators", "no");

    parser.add_list_option<shared_ptr<SubtaskGenerator>>(
        "subtasks",
        "subtask generators",
        "[landmarks(),goals()]");
    parser.add_option<int>(
        "max_states",
        "maximum sum of abstract states over all abstractions",
        "infinity",
        Bounds("1", "infinity"));
    parser.add_option<int>(
        "max_transitions",
        "maximum sum of real transitions (excluding self-loops) over "
        " all abstractions",
        "1M",
        Bounds("0", "infinity"));
    parser.add_option<double>(
        "max_time",
        "maximum time in seconds for building abstractions",
        "infinity",
        Bounds("0.0", "infinity"));
    vector<string> pick_strategies;
    pick_strategies.push_back("RANDOM");
    pick_strategies.push_back("MIN_UNWANTED");
    pick_strategies.push_back("MAX_UNWANTED");
    pick_strategies.push_back("MIN_REFINED");
    pick_strategies.push_back("MAX_REFINED");
    pick_strategies.push_back("MIN_HADD");
    pick_strategies.push_back("MAX_HADD");
    parser.add_enum_option<PickSplit>(
        "pick", pick_strategies, "split-selection strategy", "MAX_REFINED");
    parser.add_option<bool>(
        "use_general_costs",
        "allow negative costs in cost partitioning",
        "true");
    Heuristic::add_options_to_parser(parser);
    utils::add_rng_options(parser);

    Options opts = parser.parse();

    if (parser.dry_run())
        return nullptr;

    return make_shared<OSPCartesianHeuristic>(opts);
}

static Plugin<Evaluator> _plugin("ospcegar", _parse);
}
