#include "heuristic_descend_policy.h"

#include "../../evaluation_context.h"
#include "../../evaluator.h"
#include "../../option_parser.h"
#include "../../plugin.h"

#include <limits>
#include <vector>

namespace policy_testing {
HeuristicDescendPolicy::HeuristicDescendPolicy(const options::Options &opts)
    : Policy(opts)
      , heuristic_(opts.get<std::shared_ptr<Evaluator>>("eval"))
      , strictly_descend_(opts.get<bool>("strictly_descend"))
      , stop_at_dead_ends_(opts.get<bool>("stop_at_dead_ends")) {
}

void
HeuristicDescendPolicy::add_options_to_parser(options::OptionParser &parser) {
    Policy::add_options_to_parser(parser);
    parser.add_option<std::shared_ptr<Evaluator>>("eval");
    parser.add_option<bool>("strictly_descend", "", "false");
    parser.add_option<bool>("stop_at_dead_ends", "", "true");
}

OperatorID
HeuristicDescendPolicy::apply(const State &state) {
    int h0 = std::numeric_limits<int>::max();
    if (strictly_descend_ || stop_at_dead_ends_) {
        EvaluationContext ctxt(state);
        EvaluationResult r = heuristic_->compute_result(ctxt);
        if (!r.is_infinite()) {
            h0 = r.get_evaluator_value();
        } else if (stop_at_dead_ends_) {
            return NO_OPERATOR;
        }
    }
    std::vector<OperatorID> aops;
    generate_applicable_ops(state, aops);
    int best = -1;
    int h_best = strictly_descend_ ? h0 : std::numeric_limits<int>::max();
    for (unsigned i = 0; i < aops.size(); ++i) {
        State succ = get_successor_state(state, aops[i]);
        EvaluationContext ctxt(succ);
        EvaluationResult r = heuristic_->compute_result(ctxt);
        if (r.get_evaluator_value() < h_best) {
            best = i;
            h_best = r.get_evaluator_value();
        }
    }
    return best < 0 ? NO_OPERATOR : aops[best];
}

static Plugin<Policy> _plugin(
    "heuristic_descend_policy",
    options::parse<Policy, HeuristicDescendPolicy>);
} // namespace policy_testing
