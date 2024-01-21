#include "goal_subset_astar.h"

#include "../evaluator.h"
#include "../open_list_factory.h"
#include "../option_parser.h"
#include "../pruning_method.h"

#include "../algorithms/ordered_set.h"
#include "../task_utils/successor_generator.h"

#include "../utils/logging.h"

#include "../../search_engines/search_common.h"

#include "../../option_parser.h"
#include "../../plugin.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <optional.hh>
#include <set>

using namespace std;
using namespace policy_pruning_method;

namespace goal_subset_astar {
GoalSubsetAStar::GoalSubsetAStar(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      anytime(opts.get<bool>("anytime")),
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
      eval(opts.get<shared_ptr<Evaluator>>("eval", nullptr)),
    pruning_method(opts.get<shared_ptr<PolicyPruningMethod>>("pruning")) {
}

void GoalSubsetAStar::initialize() {
    log << "Conducting best first branch and bound search"
        << (reopen_closed_nodes ? " with" : " without")
        << " reopening closed nodes, (real) bound = " << bound
        << endl;
    assert(open_list);

    current_msgs = MSGSCollection(anytime);
    if(! current_msgs.is_initialized()){
        current_msgs.initialize(task);
    }

    State initial_state = state_registry.get_initial_state();

    pruning_method->notify_initial_state(initial_state);

    current_msgs.track(initial_state);

    cout << "check initial state " << endl;

    MSGSEvaluationContext eval_context(initial_state, 0, true, &statistics, &current_msgs, bound);

    statistics.inc_evaluated_states();

    if (open_list->is_dead_end(eval_context)) {
        log << "Initial state is a dead end." << endl;
    } else {
        if (search_progress.check_progress(eval_context))
            statistics.print_checkpoint_line(0);
        start_f_value_statistics(eval_context);
        SearchNode node = search_space.get_node(initial_state);
        node.open_initial();

        open_list->insert(eval_context, initial_state.get_id());
    }

    print_initial_evaluator_values(eval_context);

}

void GoalSubsetAStar::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    current_msgs.print();
    pruning_method->print_statistics();
}

SearchStatus GoalSubsetAStar::step() {
    // cout << "===================== STEP =====================" << endl;
    tl::optional<SearchNode> node;
    while (true) {
        if (open_list->empty()) {
            if (osp) {
                StateID best_state_id = current_msgs.get_cardinally_best_state();
                if (best_state_id != StateID::no_state){
                    cout << "OSP task plan found" << endl;
                    State best_state = state_registry.lookup_state(best_state_id);
                    cout << "Maximal number of solved soft goals: " << current_msgs.get_max_solved_soft_goals() << endl;
                    set_osp_plan(best_state);
                    return SOLVED;
                }
            }
            else{
                if (log.is_at_least_normal())
                log << "Completely explored state space -- no solution!" << endl;
            }
            return FAILED;
        }
        StateID id = open_list->remove_min();
        // cout << "Parent State: " << id << endl;
        State s = state_registry.lookup_state(id);
        node.emplace(search_space.get_node(s));

        if (node->is_closed())
            continue;

        current_msgs.track(s);

        /*
          We can pass calculate_preferred=false here since preferred
          operators are computed when the state is expanded.
        */
        MSGSEvaluationContext eval_context(s, node->get_g(), false, &statistics, &current_msgs, bound);

        node->close();
        assert(!node->is_dead_end());
        update_f_value_statistics(eval_context);
        statistics.inc_expanded();
        break;
    }

    const State &s = node->get_state();
    if (check_goal_and_set_plan(s))
        return SOLVED;

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(s, applicable_ops);
    pruning_method->prune_operators(s, applicable_ops);

    // This evaluates the expanded state (again) to get preferred ops
    MSGSEvaluationContext eval_context(s, node->get_g(), false, &statistics, &current_msgs, bound, true);

    // int pre_estimate = eval_context.get_evaluator_value_or_infinity(eval.get());

    for (OperatorID op_id : applicable_ops) {
        // cout << "****************** EXPAND *****************" << endl;
        OperatorProxy op = task_proxy.get_operators()[op_id];
        // cout << "g = " << (node->get_real_g() + op.get_cost()) << endl;
        if ((node->get_real_g() + op.get_cost()) >= bound){
            continue;
        }
        State succ_state = state_registry.get_successor_state(s, op);
        // cout << "Succ State: " << succ_state.get_id() << endl;
        statistics.inc_generated();

        SearchNode succ_node = search_space.get_node(succ_state);

        pruning_method->notify_state_transition(s, op_id, succ_state);

        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end()){
            continue;
        }

        if (succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.

            // Careful: succ_node.get_g() is not available here yet,
            // hence the stupid computation of succ_g.
            // TODO: Make this less fragile.
            int succ_g = node->get_g() + get_adjusted_cost(op);

            MSGSEvaluationContext succ_eval_context(
                succ_state, succ_g, true, &statistics, &current_msgs, bound);
            statistics.inc_evaluated_states();

            // cout << "new goals reachable?????" << endl;
            if(succ_eval_context.is_evaluator_value_infinite(eval.get())){
                // cout << "Prune Succ State: " << succ_state.get_id() << endl;
                continue;
            }

            if (open_list->is_dead_end(succ_eval_context)) {
                // cout << "DEADEND Succ State: " << succ_state.get_id() << endl;
                succ_node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }

            succ_node.open(*node, op, get_adjusted_cost(op));

            open_list->insert(succ_eval_context, succ_state.get_id());
            if (search_progress.check_progress(succ_eval_context)) {
                statistics.print_checkpoint_line(succ_node.get_g());
                reward_progress();
            }
        } else if (succ_node.get_g() > node->get_g() + get_adjusted_cost(op)) {
            // cout << "---> new cheapest path to " << succ_state.get_id() << endl;
            // We found a new cheapest path to an open or closed state.
            if (reopen_closed_nodes) {
                if (succ_node.is_closed()) {
                    /*
                      TODO: It would be nice if we had a way to test
                      that reopening is expected behaviour, i.e., exit
                      with an error when this is something where
                      reopening should not occur (e.g. A* with a
                      consistent heuristic).
                    */
                    statistics.inc_reopened();
                }
                succ_node.reopen(*node, op, get_adjusted_cost(op));

                MSGSEvaluationContext succ_eval_context(
                    succ_state, succ_node.get_g(), true, &statistics, &current_msgs, bound);

                if(succ_eval_context.is_evaluator_value_infinite(eval.get())){
                    // cout << "Prune Succ State: " << succ_state.get_id() << endl;
                    continue;
                }

                /*
                  Note: our old code used to retrieve the h value from
                  the search node here. Our new code recomputes it as
                  necessary, thus avoiding the incredible ugliness of
                  the old "set_evaluator_value" approach, which also
                  did not generalize properly to settings with more
                  than one evaluator.

                  Reopening should not happen all that frequently, so
                  the performance impact of this is hopefully not that
                  large. In the medium term, we want the evaluators to
                  remember evaluator values for states themselves if
                  desired by the user, so that such recomputations
                  will just involve a look-up by the Evaluator object
                  rather than a recomputation of the evaluator value
                  from scratch.
                */
                open_list->insert(succ_eval_context, succ_state.get_id());
            } else {
                // If we do not reopen closed nodes, we just update the parent pointers.
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back.
                succ_node.update_parent(*node, op, get_adjusted_cost(op));
            }
        }
    }

    return IN_PROGRESS;
}

void GoalSubsetAStar::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void GoalSubsetAStar::dump_search_space() const {
    search_space.dump(task_proxy);
}

void GoalSubsetAStar::start_f_value_statistics(MSGSEvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void GoalSubsetAStar::update_f_value_statistics(MSGSEvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

void  GoalSubsetAStar::init_msgs(MSGSCollection collection){
    this->current_msgs = collection;
}

MSGSCollection  GoalSubsetAStar::get_msgs(){
    return this->current_msgs;
}

void add_options_to_parser(OptionParser &parser) {
    // SearchEngine::add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);
}

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Greedy branch and bound",
        "We break ties using the evaluator. Closed nodes are re-opened.");
    parser.add_option<bool>("anytime", "print every new MSGS that is found during search", "false");

    parser.add_option<shared_ptr<Evaluator>>("eval", "evaluator for pruning");
    parser.add_list_option<shared_ptr<Evaluator>>("evals", "evaluators");
    parser.add_list_option<shared_ptr<Evaluator>>("preferred",
    "use preferred operators of these evaluators", "[]");
    parser.add_option<int>("boost",
    "boost value for preferred operator open lists", "0");

    parser.add_option<shared_ptr<PolicyPruningMethod>>("pruning", "TODO");

    add_options_to_parser(parser);
    Options opts = parser.parse();
    opts.verify_list_non_empty<shared_ptr<Evaluator>>("evals");

    shared_ptr<GoalSubsetAStar> engine;
    if (!parser.dry_run()) {
        opts.set("open", search_common::create_greedy_open_list_factory(opts));
        opts.set("reopen_closed", true);
        engine = make_shared<GoalSubsetAStar>(opts);
    }

    return engine;
}

static Plugin<SearchEngine> _plugin("gsastar", _parse);

}
