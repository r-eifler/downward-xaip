#include "hill_climbing_policy.h"

#include "../../evaluation_context.h"
#include "../../evaluator.h"
#include "../../option_parser.h"
#include "../../plugin.h"
#include "../../task_utils/task_properties.h"
#include "../out_of_resource_exception.h"

#include <deque>
#include <vector>

namespace policy_testing {
HillClimbingPolicy::HillClimbingPolicy(const options::Options &opts)
    : Policy(opts)
      , heuristic_(opts.get<std::shared_ptr<Evaluator>>("eval"))
      , helpful_actions_pruning_(opts.get<bool>("helpful_actions_pruning")) {
}

void
HillClimbingPolicy::add_options_to_parser(options::OptionParser &parser) {
    Policy::add_options_to_parser(parser);
    parser.add_option<std::shared_ptr<Evaluator>>("eval");
    parser.add_option<bool>("helpful_actions_pruning", "", "false");
}

OperatorID
HillClimbingPolicy::apply(const State &state0) {
    struct ParentInfo {
        StateID parent_id;
        int op_idx;
        explicit ParentInfo()
            : parent_id(StateID::no_state)
              , op_idx(-1) {
        }
        explicit ParentInfo(StateID p, int o)
            : parent_id(p)
              , op_idx(o) {
        }
    };

    utils::HashMap<StateID, ParentInfo> closed;
    closed.emplace(
        std::pair<StateID, ParentInfo>(state0.get_id(), ParentInfo()));

    std::deque<State> open;
    open.push_back(state0);

    int to_beat = -1;

    std::vector<OperatorID> aops;

    StateID exit = StateID::no_state;

    while (!open.empty()) {
        State state = open.front();
        open.pop_front();
        // evaluate only upon expansion as otherwise heuristic would need to be
        // recomputed if helpful actions pruning is activated
        EvaluationContext context(state, nullptr, true);
        const EvaluationResult h = heuristic_->compute_result(context);
        if (h.is_infinite()) {
            continue;
        }
        if (to_beat == -1) {
            to_beat = h.get_evaluator_value();
        } else if (h.get_evaluator_value() < to_beat) {
            exit = state.get_id();
            break;
        }
        if (are_limits_reached()) {
            throw OutOfResourceException();
        }
        // force policy to be deterministic
        const OperatorID chosen_before = can_lookup_action(state) ? lookup_action(state) : NO_OPERATOR;
        if (chosen_before != NO_OPERATOR) {
            aops.push_back(chosen_before);
        } else if (helpful_actions_pruning_) {
            aops = h.get_preferred_operators();
        } else {
            generate_applicable_ops(state, aops);
        }
        for (auto &aop : aops) {
            State succ = get_successor_state(state, aop);
            if (closed
                .emplace(std::pair<StateID, ParentInfo>(
                             succ.get_id(),
                             ParentInfo(state.get_id(), aop.get_index())))
                .second) {
                if (task_properties::is_goal_state(get_task_proxy(), succ)) {
                    exit = succ.get_id();
                    open.clear();
                    break;
                }
                open.push_back(succ);
            }
        }
        aops.clear();
    }

    if (exit == StateID::no_state) {
        return NO_OPERATOR;
    }

    OperatorID result = NO_OPERATOR;
    while (exit != state0.get_id()) {
        assert(closed.find(exit) != closed.end());
        const ParentInfo &info = closed[exit];
        store_operator(
            get_state_registry().lookup_state(info.parent_id),
            OperatorID(info.op_idx));
        result = OperatorID(info.op_idx);
        exit = info.parent_id;
    }
    return result;
}

static Plugin<Policy>
_plugin("hill_climbing_policy", options::parse<Policy, HillClimbingPolicy>);
} // namespace policy_testing
