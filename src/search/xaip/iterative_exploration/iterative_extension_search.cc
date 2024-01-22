#include "iterative_extension_search.h"

#include "../explicit_mugs_search/msgs_evaluation_context.h"
#include "../../evaluator.h"
#include "../../open_list_factory.h"
#include "../../option_parser.h"
#include "../../pruning_method.h"
#include "../../plugin.h"
#include "../../search_engines/search_common.h"

#include "../../algorithms/ordered_set.h"
#include "../../task_utils/successor_generator.h"

#include "../../utils/logging.h"

#include "../goal_subsets/goal_subset.h"
#include "../goal_subsets/goal_subsets.h"
#include "../goal_subsets/output_handler.h"
#include "../explicit_mugs_search/msgs_collection.h"

#include <cassert>
#include <cstdlib>
#include <memory>
#include <optional.hh>
#include <set>

using namespace std;
using namespace goalsubset;
using namespace policy_pruning_method;

namespace iterative_extension_search {
IterativeExtensionSearch::IterativeExtensionSearch(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
        eval(opts.get<shared_ptr<Evaluator>>("eval", nullptr)),
        pruning_method(opts.get<shared_ptr<PolicyPruningMethod>>("pruning")),
        radius_tracker(RadiusTracker(
            opts.get<double>("min_radius"),
            opts.get<double>("max_radius"),
            opts.get<double>("step_size")
        )) {
    
}

void IterativeExtensionSearch::initialize() {
    log << "Conducting best first search"
        << (reopen_closed_nodes ? " with" : " without")
        << " reopening closed nodes, (real) bound = " << bound
        << endl;
    assert(open_list);

    if(! current_msgs.is_initialized()){
        current_msgs.initialize(task);
    }

    State initial_state = state_registry.get_initial_state();

    pruning_method->notify_initial_state(initial_state);

    current_msgs.track(initial_state);
    
    current_radius = radius_tracker.next_radius();
    cout << "##############################################################################" << endl;
    cout << "##############################################################################" << endl;
    cout << "######################## radius:" << current_radius << " ####################################" << endl;

    /*
      Note: we consider the initial state as reached by a preferred
      operator.
    */
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

void IterativeExtensionSearch::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    // pruning_method->print_statistics();
    // taskRelaxationTracker->results_to_file();
}

void IterativeExtensionSearch::expand(const State &state){

    tl::optional<SearchNode> node;
    node.emplace(search_space.get_node(state));

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(state, applicable_ops);
    std::vector<pair<float,OperatorID>> to_postpone =  pruning_method->postpone(state, applicable_ops);


    for (pair<float,OperatorID> dis_op : to_postpone) {
        radius_tracker.add_frontier_state(dis_op.first, state.get_id(), dis_op.second);
    }

    for (OperatorID op_id : applicable_ops) {
        decide_to_openlist(*node, state, op_id);
    }

}

bool IterativeExtensionSearch::decide_to_openlist(const SearchNode &node, const State &state, OperatorID op_id){
    OperatorProxy op = task_proxy.get_operators()[op_id];
    if ((node.get_real_g() + op.get_cost()) >= bound)
        return false;

    State succ_state = state_registry.get_successor_state(state, op);
    statistics.inc_generated();

    SearchNode succ_node = search_space.get_node(succ_state);

    // Previously encountered dead end. Don't re-evaluate.
    if (succ_node.is_dead_end())
        return false;

    if (succ_node.is_new()) {
        pruning_method->notify_state_transition(state, op_id, succ_state);
        // We have not seen this state before.
        // Evaluate and create a new node.

        // Careful: succ_node.get_g() is not available here yet,
        // hence the stupid computation of succ_g.
        // TODO: Make this less fragile.
        int succ_g = node.get_g() + get_adjusted_cost(op);

        MSGSEvaluationContext succ_eval_context(
            succ_state, succ_g, true, &statistics, &current_msgs, bound);
        statistics.inc_evaluated_states();

        if (open_list->is_dead_end(succ_eval_context)) {
            succ_node.mark_as_dead_end();
            statistics.inc_dead_ends();
            return false;
        }

        if(succ_eval_context.is_evaluator_value_infinite(eval.get())){
            // cout << "Prune Succ State: " << succ_state.get_id() << endl;
            return false;
        }

        succ_node.open(node, op, get_adjusted_cost(op));

        open_list->insert(succ_eval_context, succ_state.get_id());
        if (search_progress.check_progress(succ_eval_context)) {
            statistics.print_checkpoint_line(succ_node.get_g());
            reward_progress();
        }
        return true;
    } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(op)) {
        pruning_method->notify_state_transition(state, op_id, succ_state);
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
            succ_node.reopen(node, op, get_adjusted_cost(op));

            MSGSEvaluationContext succ_eval_context(
                succ_state, succ_node.get_g(), true, &statistics, &current_msgs, bound);

            if(succ_eval_context.is_evaluator_value_infinite(eval.get())){
                // cout << "Prune Succ State: " << succ_state.get_id() << endl;
                return false;
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
            succ_node.update_parent(node, op, get_adjusted_cost(op));
            return false;
        }
    }
    return false;
}


SearchStatus IterativeExtensionSearch::step() {
    tl::optional<SearchNode> node;
    while (true) {
        if (open_list->empty()) {
            // if (log.is_at_least_normal())
            //     log << "Completely explored state space -- no solution!" << endl;
            if (! next_radius()){
                std::cout << "no larger radii or nothing left to explore" << std::endl;
                return SearchStatus::FAILED;
            }
        }
        StateID id = open_list->remove_min();
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
    if (check_goal_and_set_plan(s)){
        return SearchStatus::SOLVED;
    }

    expand(s);

    return IN_PROGRESS;
}

void IterativeExtensionSearch::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void IterativeExtensionSearch::dump_search_space() const {
    search_space.dump(task_proxy);
}

void IterativeExtensionSearch::start_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void IterativeExtensionSearch::update_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

bool IterativeExtensionSearch::next_radius() {

    while(true){

        statistics.print_detailed_statistics();
        current_msgs.print();

        cout << "##############################################################################" << endl;
        cout << "##############################################################################" << endl;
        if (! radius_tracker.has_next_radius()){
            return false;
        }

        current_radius = radius_tracker.next_radius();
        pruning_method->set_radius(current_radius);
        cout << "radius:" << current_radius << endl;
        cout << "frontier size: " << radius_tracker.get_frontier_size(current_radius) << endl;

        bool something_to_explored = false;
        for(FrontierElem elem : radius_tracker.get_frontier_states(current_radius)){
            State parent = state_registry.lookup_state(elem.parent);
            SearchNode parent_node = search_space.get_node(parent);

            something_to_explored |= decide_to_openlist(parent_node, parent, elem.op);
        }

        if (something_to_explored){
            break;
        }
    }

    return true;
}


void add_options_to_parser(OptionParser &parser) {
    // SearchEngine::add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);
}

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Greedy branch and bound",
        "We break ties using the evaluator. Closed nodes are re-opened.");

    parser.add_option<double>("min_radius", "TODO", "0");
    parser.add_option<double>("max_radius", "TODO", "1");
    parser.add_option<double>("step_size", "TODO", "1");

    parser.add_option<shared_ptr<Evaluator>>("eval", "evaluator for pruning");
    parser.add_list_option<shared_ptr<Evaluator>>("evals", "evaluators");
    parser.add_list_option<shared_ptr<Evaluator>>("preferred",
        "use preferred operators of these evaluators", "[]");
    parser.add_option<int>("boost",
        "boost value for preferred operator open lists", "0");

    parser.add_option<shared_ptr<PolicyPruningMethod>>("pruning", "TODO");

    IterativeExtensionSearch::add_options_to_parser(parser);
    Options opts = parser.parse();
    opts.verify_list_non_empty<shared_ptr<Evaluator>>("evals");

    shared_ptr<IterativeExtensionSearch> engine;
    if (!parser.dry_run()) {
        opts.set("open", search_common::create_greedy_open_list_factory(opts));
        opts.set("reopen_closed", true);
        engine = make_shared<IterativeExtensionSearch>(opts);
    }

    return engine;
}

static Plugin<SearchEngine> _plugin("policy_iterative_extension", _parse);
}
