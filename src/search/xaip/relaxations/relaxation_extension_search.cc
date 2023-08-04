#include "relaxation_extension_search.h"

#include "../../evaluation_context.h"
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
      f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)),
      preferred_operator_evaluators(opts.get_list<shared_ptr<Evaluator>>("preferred")),
      lazy_evaluator(opts.get<shared_ptr<Evaluator>>("lazy_evaluator", nullptr)),
      pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")) {
    if (lazy_evaluator && !lazy_evaluator->does_cache_estimates()) {
        cerr << "lazy_evaluator must cache its estimates" << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
}

void RelaxationExtensionSearch::initialize() {
    log << "Conducting best first search"
        << (reopen_closed_nodes ? " with" : " without")
        << " reopening closed nodes, (real) bound = " << bound
        << endl;
    assert(open_list);

    set<Evaluator *> evals;
    open_list->get_path_dependent_evaluators(evals);

    /*
      Collect path-dependent evaluators that are used for preferred operators
      (in case they are not also used in the open list).
    */
    for (const shared_ptr<Evaluator> &evaluator : preferred_operator_evaluators) {
        evaluator->get_path_dependent_evaluators(evals);
    }

    /*
      Collect path-dependent evaluators that are used in the f_evaluator.
      They are usually also used in the open list and will hence already be
      included, but we want to be sure.
    */
    if (f_evaluator) {
        f_evaluator->get_path_dependent_evaluators(evals);
    }

    /*
      Collect path-dependent evaluators that are used in the lazy_evaluator
      (in case they are not already included).
    */
    if (lazy_evaluator) {
        lazy_evaluator->get_path_dependent_evaluators(evals);
    }

    path_dependent_evaluators.assign(evals.begin(), evals.end());

    pruning_method->initialize(task);

    State initial_state = state_registry.get_initial_state();
    for (Evaluator *evaluator : path_dependent_evaluators) {
        evaluator->notify_initial_state(initial_state);
    }

    pruning_method->prune_state(initial_state, bound);
    
    taskRelaxationTracker = new TaskRelaxationTracker(this->getTask());
    relaxedTask = taskRelaxationTracker->next_relaxed_task();
    cout << "initial relaxed task: " << relaxedTask->get_name() << endl;

    /*
      Note: we consider the initial state as reached by a preferred
      operator.
    */
    EvaluationContext eval_context(initial_state, 0, true, &statistics);

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
        output_handler.add_goal_subsets(task->get_name(), task->get_msgs().get_mugs());
    }
    output_handler.output();
}

bool RelaxationExtensionSearch::expand(const State &state){

    bool succ_state_generated = false;

    tl::optional<SearchNode> node;
    node.emplace(search_space.get_node(state));

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(state, applicable_ops);

    /*
      TODO: When preferred operators are in use, a preferred operator will be
      considered by the preferred operator queues even when it is pruned.
    */
    pruning_method->prune_operators(state, applicable_ops);

    // This evaluates the expanded state (again) to get preferred ops
    EvaluationContext eval_context(state, node->get_g(), false, &statistics, true);
    ordered_set::OrderedSet<OperatorID> preferred_operators;
    for (const shared_ptr<Evaluator> &preferred_operator_evaluator : preferred_operator_evaluators) {
        collect_preferred_operators(eval_context,
                                    preferred_operator_evaluator.get(),
                                    preferred_operators);
    }

    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];
        if ((node->get_real_g() + op.get_cost()) >= bound)
            continue;

        State succ_state = state_registry.get_successor_state(state, op);
        statistics.inc_generated();
        bool is_preferred = preferred_operators.contains(op_id);

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

            EvaluationContext succ_eval_context(
                succ_state, succ_g, is_preferred, &statistics);
            statistics.inc_evaluated_states();

            if (open_list->is_dead_end(succ_eval_context)) {
                succ_node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }
            if (pruning_method->prune_state(succ_state, bound - (node->get_real_g() + op.get_cost()))){
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

                EvaluationContext succ_eval_context(
                    succ_state, succ_node.get_g(), is_preferred, &statistics);

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

        /*
          We can pass calculate_preferred=false here since preferred
          operators are computed when the state is expanded.
        */
        EvaluationContext eval_context(s, node->get_g(), false, &statistics);

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
        relaxedTask->propagate_solvable(relaxedTask->get_msgs());
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
    relaxedTask->add_msgs(pruning_method->get_msgs());
    if (previous_relaxedTask){
        relaxedTask->set_num_expanded_states(statistics.get_expanded() - previous_relaxedTask->get_num_expanded_states());
    } 
    else {
        relaxedTask->set_num_expanded_states(statistics.get_expanded());
    }
    relaxedTask->print();
    statistics.print_detailed_statistics();



    cout << "##############################################################################" << endl;
    cout << "##############################################################################" << endl;
    if (! taskRelaxationTracker->has_next_relaxed_task()){
        return false;
    }

    previous_relaxedTask = relaxedTask;
    relaxedTask = taskRelaxationTracker->next_relaxed_task();
    cout << "Current task: " << relaxedTask->get_name() << endl;

    for (RelaxedTask* t : relaxedTask->get_lower_cover()){
        relaxedTask->add_msgs(t->get_msgs());
    }

    pruning_method->init_msgs(relaxedTask->get_msgs());
    cout << "init #MSGS: " << relaxedTask->get_msgs().size() << endl;

    return init_with_frontier_states();
}

bool RelaxationExtensionSearch::init_with_frontier_states()
{
    std::cout << "EXTERNAL RELAXATION: continue with frontier states" << std::endl;
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
        "A* search (eager)",
        "A* is a special case of eager best first search that uses g+h "
        "as f-function. "
        "We break ties using the evaluator. Closed nodes are re-opened.");
    parser.add_option<shared_ptr<Evaluator>>("eval", "evaluator for h-value");
    parser.add_option<shared_ptr<Evaluator>>(
        "lazy_evaluator",
        "An evaluator that re-evaluates a state before it is expanded.",
        OptionParser::NONE);

    relaxation_extension_search::add_options_to_parser(parser);
    Options opts = parser.parse();

    shared_ptr<relaxation_extension_search::RelaxationExtensionSearch> engine;
    if (!parser.dry_run()) {
        auto temp = search_common::create_astar_open_list_factory_and_f_eval(opts);
        opts.set("open", temp.first);
        opts.set("f_eval", temp.second);
        opts.set("reopen_closed", true);
        vector<shared_ptr<Evaluator>> preferred_list;
        opts.set("preferred", preferred_list);
        engine = make_shared<relaxation_extension_search::RelaxationExtensionSearch>(opts);
    }

    return engine;
}

static Plugin<SearchEngine> _plugin("astar_relaxations", _parse);
}
