#include "relaxation_extension_search.h"

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

namespace relaxation_extension_search {
RelaxationExtensionSearch::RelaxationExtensionSearch(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
        eval(opts.get<shared_ptr<Evaluator>>("eval", nullptr)){
    
}

void RelaxationExtensionSearch::initialize() {
    log << "Conducting best first search"
        << (reopen_closed_nodes ? " with" : " without")
        << " reopening closed nodes, (real) bound = " << bound
        << endl;
    assert(open_list);

    if(! current_msgs.is_initialized()){
        current_msgs.initialize(task);
    }

    State initial_state = state_registry.get_initial_state();

    current_msgs.track(initial_state);
    
    taskRelaxationTracker = new TaskRelaxationTracker(this->getTask());
    relaxedTask = taskRelaxationTracker->next_relaxed_task();
    cout << "initial relaxed task: " << relaxedTask->get_name() << endl;

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

void RelaxationExtensionSearch::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    // pruning_method->print_statistics();
    // taskRelaxationTracker->results_to_file();
}

bool RelaxationExtensionSearch::expand(const State &state){

    bool succ_state_generated = false;

    tl::optional<SearchNode> node;
    node.emplace(search_space.get_node(state));

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(state, applicable_ops);

    // This evaluates the expanded state (again) to get preferred ops
    MSGSEvaluationContext eval_context(state, node->get_g(), false, &statistics, &current_msgs, true);

    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];
        if ((node->get_real_g() + op.get_cost()) >= bound)
            continue;

        State succ_state = state_registry.get_successor_state(state, op);
        statistics.inc_generated();

        //for external relaxation check whether the state satisfies the limits
        if (! relaxedTask->sat_limits(succ_state)){
            relaxedTask->add_frontier_state(succ_state.get_id());
            continue;
        }

        SearchNode succ_node = search_space.get_node(succ_state);

        for (Evaluator *evaluator : path_dependent_evaluators) {
            evaluator->notify_state_transition(state, op_id, succ_state);
        }

        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end())
            continue;

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

            if (open_list->is_dead_end(succ_eval_context)) {
                succ_node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }

            if(succ_eval_context.is_evaluator_value_infinite(eval.get())){
                // cout << "Prune Succ State: " << succ_state.get_id() << endl;
                continue;
            }

            succ_node.open(*node, op, get_adjusted_cost(op));

            succ_state_generated = true;
            open_list->insert(succ_eval_context, succ_state.get_id());
            if (search_progress.check_progress(succ_eval_context)) {
                statistics.print_checkpoint_line(succ_node.get_g());
                reward_progress();
            }
        } else if (succ_node.get_g() > node->get_g() + get_adjusted_cost(op)) {
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
                succ_state_generated = true;
                open_list->insert(succ_eval_context, succ_state.get_id());
            } else {
                // If we do not reopen closed nodes, we just update the parent pointers.
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back.
                succ_node.update_parent(*node, op, get_adjusted_cost(op));
            }
        }
    }
    return succ_state_generated;
}

SearchStatus RelaxationExtensionSearch::step() {
    tl::optional<SearchNode> node;
    while (true) {
        if (open_list->empty()) {
            // if (log.is_at_least_normal())
            //     log << "Completely explored state space -- no solution!" << endl;
            if (! next_relaxed_task()){
                std::cout << "no more relaxed tasks" << std::endl;
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

        if (lazy_evaluator) {
            /*
              With lazy evaluators (and only with these) we can have dead nodes
              in the open list.

              For example, consider a state s that is reached twice before it is expanded.
              The first time we insert it into the open list, we compute a finite
              heuristic value. The second time we insert it, the cached value is reused.

              During first expansion, the heuristic value is recomputed and might become
              infinite, for example because the reevaluation uses a stronger heuristic or
              because the heuristic is path-dependent and we have accumulated more
              information in the meantime. Then upon second expansion we have a dead-end
              node which we must ignore.
            */
            if (node->is_dead_end())
                continue;

            if (lazy_evaluator->is_estimate_cached(s)) {
                int old_h = lazy_evaluator->get_cached_estimate(s);
                int new_h = eval_context.get_evaluator_value_or_infinity(lazy_evaluator.get());
                if (open_list->is_dead_end(eval_context)) {
                    node->mark_as_dead_end();
                    statistics.inc_dead_ends();
                    continue;
                }
                if (new_h != old_h) {
                    open_list->insert(eval_context, id);
                    continue;
                }
            }
        }

        node->close();
        assert(!node->is_dead_end());
        update_f_value_statistics(eval_context);
        statistics.inc_expanded();
        break;
    }

    const State &s = node->get_state();
    if (check_goal_and_set_plan(s)){
        relaxedTask->set_solvable(true);
        if (! next_relaxed_task()) {
            return SearchStatus::SOLVED;
        }
        else{
            // clear call stack to start with a new set of initial states
            open_list->clear();
            return SearchStatus::IN_PROGRESS;
        }
    }

    expand(s);

    return IN_PROGRESS;
}

void RelaxationExtensionSearch::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void RelaxationExtensionSearch::dump_search_space() const {
    search_space.dump(task_proxy);
}

void RelaxationExtensionSearch::start_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void RelaxationExtensionSearch::update_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

bool RelaxationExtensionSearch::next_relaxed_task() {
    while(!pending_initial_states.empty()){
//        if (pending_initial_states.size() % 10 == 0)
//            std::cout << "#pending init states: " << pending_initial_states.size() << std::endl;
        StateID id = pending_initial_states.back();
        pending_initial_states.pop_back();
        State s = state_registry.lookup_state(id);
        SearchNode inode = search_space.get_node(s);
        inode.open_initial();
        if (expand(s)){
            return true;
        }
    }

    // update MSGS of finished iteration
    relaxedTask->add_msgs(current_msgs);

    relaxedTask->set_num_expanded_states(statistics.get_expanded() - expanded_states_up_to_prev_task);
    expanded_states_up_to_prev_task = statistics.get_expanded();

    relaxedTask->print();
    statistics.print_detailed_statistics();

    relaxedTask->propagate_solvable();



    cout << "##############################################################################" << endl;
    cout << "##############################################################################" << endl;
    if (! taskRelaxationTracker->has_next_relaxed_task()){
        return false;
    }

    relaxedTask = taskRelaxationTracker->next_relaxed_task();
    cout << "Current task: " << relaxedTask->get_name() << endl;

    for (RelaxedTask* t : relaxedTask->get_lower_cover()){
        relaxedTask->add_msgs(t->get_msgs());
    }

    current_msgs = relaxedTask->get_msgs();
    cout << "init #MSGS: " << relaxedTask->get_msgs().size() << endl;

    return init_with_frontier_states();
}

bool RelaxationExtensionSearch::init_with_frontier_states()
{
    std::cout << "ITERATIVE EXPANSION OF STATE SPACE: continue with frontier states" << std::endl;
//    std::cout << "frontier: #states: " << pending_initial_states.size() << std::endl;

    //Get frontier states from lower cover
    std::unordered_set<StateID> init_frontier;
    int num_init_states = 0;
    int num_frontier_states = 0;
    int num_skipped = 0;
    vector<RelaxedTask*> lower_cover = relaxedTask->get_lower_cover();
    for (int i = 0; i < (int) lower_cover.size(); i++){
        RelaxedTask* t = lower_cover[i];
        std::unordered_set<StateID> t_frontier = t->get_frontier();
        cout << t->get_name() << " #frontier states: " << t_frontier.size() << endl;
        for(StateID id : t_frontier){
            State s = state_registry.lookup_state(id);
            bool skip = false;
            // check whether state is sat by any other covering task, then you can skip it
            for (int j = 0; j < (int) lower_cover.size(); j++){
                if (i != j){
                    if (lower_cover[j]->sat_limits(s)){
                        skip = true;
                        break;
                    }
                }
            }
            if (skip){
                num_skipped++;
                continue;
            }
            if (relaxedTask->sat_limits(s)){
                init_frontier.insert(id);
                num_init_states++;
            }
            else{
                relaxedTask->add_frontier_state(id);
                num_frontier_states++;
            }
        }
/*        std::cout << "#skipped states : " << num_skipped << std::endl;
        std::cout << "#init states : " << num_init_states << std::endl;
        std::cout << "#frontier states : " << num_frontier_states << std::endl;*/
    }
    pending_initial_states.insert(
            pending_initial_states.end(),
            std::make_move_iterator(init_frontier.begin()),
            std::make_move_iterator(init_frontier.end())
    );
    std::cout << "total sum: " << num_frontier_states << std::endl;
    std::cout << "#skipped states total: " << num_skipped << std::endl;
    std::cout << "stay in frontier #states: " << relaxedTask->get_frontier().size() << std::endl;
    std::cout << "Next Iteration init #states: " << pending_initial_states.size() << " with duplicates: " << num_init_states << std::endl;
    return next_relaxed_task();
}

void add_options_to_parser(OptionParser &parser) {
    SearchEngine::add_pruning_option(parser);
    SearchEngine::add_options_to_parser(parser);
}

static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Greedy branch and bound",
        "We break ties using the evaluator. Closed nodes are re-opened.");

    parser.add_option<shared_ptr<Evaluator>>("eval", "evaluator for pruning");
    parser.add_list_option<shared_ptr<Evaluator>>("evals", "evaluators");
    parser.add_list_option<shared_ptr<Evaluator>>("preferred",
        "use preferred operators of these evaluators", "[]");
    parser.add_option<int>("boost",
        "boost value for preferred operator open lists", "0");

    relaxation_extension_search::add_options_to_parser(parser);
    Options opts = parser.parse();
    opts.verify_list_non_empty<shared_ptr<Evaluator>>("evals");

    shared_ptr<relaxation_extension_search::RelaxationExtensionSearch> engine;
    if (!parser.dry_run()) {
        opts.set("open", search_common::create_greedy_open_list_factory(opts));
        opts.set("reopen_closed", true);
        engine = make_shared<RelaxationExtensionSearch>(opts);
    }

    return engine;
}

static Plugin<SearchEngine> _plugin("astar_relaxations", _parse);
}
