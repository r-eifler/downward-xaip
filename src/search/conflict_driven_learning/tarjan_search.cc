
#include "tarjan_search.h"

#include "state_component.h"

#include "../state_registry.h"
#include "../task_proxy.h"
#include "../evaluation_context.h"
#include "../evaluation_result.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../algorithms/ordered_set.h"
#include "../pruning_method.h"

#include "../option_parser.h"
#include "../plugin.h"

#include <algorithm>
#include <iostream>
#include <set>

#ifndef NDEBUG
#define NDEBUG_VERIFY_RECOGNIZED_NEIGHBORS 1
#define NDEBUG_VERIFY_REFINEMENT 1
#endif

struct StateLess {
  bool operator()(const State& s1, const State& s2) const {
      StateID i1 = s1.get_id();
      StateID i2 = s2.get_id();
      return i1(i1) < i2(i2);
  }
};

using StateSet = std::set<State, StateLess>;

namespace conflict_driven_learning
{
namespace tarjan_search
{

TarjanSearch::CallStackElement::CallStackElement(
    SearchNode &node) :
    node(node), last_successor_id(StateID::no_state), succ_result(DFSResult::FAILED)
{}

TarjanSearch::TarjanSearch(const options::Options &opts)
    : SearchEngine(opts)
    , c_recompute_u(opts.get<bool>("recompute_u"))
    , c_refine_initial_state(opts.get<bool>("refine_initial_state"))
    , c_prune_eval_dead_ends(opts.get<bool>("prune_eval_dead_ends"))
    , c_compatible_pruning_method(opts.get<bool>("compatible_pruning_method"))
    , c_dead_end_refinement(opts.contains("learn"))
    , c_compute_recognized_neighbors(c_dead_end_refinement)
    , m_guidance(opts.contains("eval") ? opts.get<std::shared_ptr<Evaluator>>("eval") : nullptr)
    , m_preferred(opts.contains("preferred") ? opts.get<std::shared_ptr<Evaluator>>("preferred") : nullptr)
    , m_learner(opts.contains("learn") ? opts.get<std::shared_ptr<ConflictLearner>>("learn") : nullptr)
    , m_dead_end_identifier(opts.contains("u_eval") ? opts.get<std::shared_ptr<Evaluator >>("u_eval") : nullptr)
    , m_pruning_method(opts.get<std::shared_ptr<PruningMethod>>("pruning"))
    , m_search_space(&state_registry)
    , m_current_index(0)
    , m_result(DFSResult::FAILED)
    , m_open_states(1)
{
    c_compute_recognized_neighbors = m_learner != nullptr && m_learner->requires_recognized_neighbors();
    if (m_guidance) {
        m_guidance->get_path_dependent_evaluators(m_path_dependent_evaluators);
    }
    if (m_preferred) {
        m_preferred->get_path_dependent_evaluators(m_path_dependent_evaluators);
    }
    if (m_dead_end_identifier) {
        m_dead_end_identifier->get_path_dependent_evaluators(m_path_dependent_evaluators);
    }
}

void TarjanSearch::initialize()
{
    std::cout << "Initializing tarjan search ..." << std::endl;
    std::cout << (c_compute_recognized_neighbors ? "M" : "Not m")
              << "aintaining recognized neighbors." << std::endl;

    m_open_states = 1;
    m_current_index = 0;
    m_current_depth = 0;
    m_result = DFSResult::FAILED;
    State istate = state_registry.get_initial_state();
    for (Evaluator* pde : m_path_dependent_evaluators) {
        pde->notify_initial_state(istate);
    }
    SearchNode inode = m_search_space[istate];
    m_pruning_method->initialize(task);
    if (!evaluate(istate)) {
        std::cout << "Initial state is dead-end!" << std::endl;
        inode.mark_recognized_dead_end();
    } else {
        inode.open_initial_state(get_h_value());
        if (task_properties::is_goal_state(task_proxy, istate)) {
            m_result = DFSResult::SOLVED;
        } else {
            expand(istate);
        }
    }
}

bool TarjanSearch::evaluate(const State& state)
{
    statistics.inc_evaluated_states();
    bool res = evaluate(state, m_guidance.get());
    if (!res && !c_prune_eval_dead_ends) {
        m_eval_result.set_evaluator_value(0);
        return true;
    }
    return res;
}

bool TarjanSearch::evaluate(const State& state, Evaluator* eval)
{
    statistics.inc_evaluations();
    if (eval == nullptr) {
        m_eval_result.set_evaluator_value(0);
        return true;
    }
    EvaluationContext ctxt(state, 0, false, nullptr);
    m_eval_result = eval->compute_result(ctxt);
    if (m_eval_result.is_infinite()) {
        return false;
    }
    return true;
}

bool TarjanSearch::evaluate_dead_end_heuristic(const State& state)
{
    if (m_dead_end_identifier == nullptr) {
        return false;
    }
    EvaluationContext ctxt(state, 0, false, nullptr);
    EvaluationResult res = m_dead_end_identifier->compute_result(ctxt);
    if (res.is_infinite()) {
        return true;
    }
    return false;
}

int TarjanSearch::get_h_value() const
{
    return m_eval_result.get_evaluator_value();
}

bool TarjanSearch::expand(const State& state)
{
    static std::vector<OperatorID> aops;
    static ordered_set::OrderedSet<OperatorID> preferred;

    SearchNode node = m_search_space[state];
    m_open_states--;

    if (evaluate_dead_end_heuristic(state)) {
        node.mark_recognized_dead_end();
        return false;
    }

#if 0
    if (m_pruning_method->prune_state(state)) {
        node.mark_dead_end();
        if (c_compatible_pruning_method) {
            node.mark_recognized_dead_end();
        }
        return false;
    }
#endif

    statistics.inc_expanded();

    node.close(m_current_index);
    m_current_index++;
    m_stack.push_front(state);
    if (c_compute_recognized_neighbors) {
        m_rn_offset.push_front(m_recognized_neighbors.size());
    }
    m_call_stack.emplace_back(node);

    m_open_list.push_layer();
    successor_generator.generate_applicable_ops(state, aops);
    unsigned num_all_aops = aops.size();
    m_pruning_method->prune_operators(state, aops);
    if (aops.size() < num_all_aops && !c_compatible_pruning_method) {
        m_call_stack.back().succ_result = DFSResult::UNRECOGNIZED;
    }

    statistics.inc_generated(aops.size());
    if (m_preferred) {
        if (evaluate(state, m_preferred.get())) {
            const std::vector<OperatorID>& pref = m_eval_result.get_preferred_operators();
            for (int i = pref.size() - 1; i >= 0; i--) {
                preferred.insert(pref[i]);
            }
        }
    }
    for (unsigned i = 0; i < aops.size(); i++) {
        State succ = state_registry.get_successor_state(
                state,
                task_proxy.get_operators()[aops[i]]);
        for (Evaluator* pde : m_path_dependent_evaluators) {
            pde->notify_state_transition(state, aops[i], succ);
        }
        SearchNode succ_node = m_search_space[succ];
        assert(succ_node.is_new() || succ_node.get_lowlink() <= succ_node.get_index());
        if (succ_node.is_new()) {
            if (evaluate(succ)) {
                succ_node.open(node, aops[i], get_h_value());
                m_open_states++;
            } else {
              statistics.inc_dead_ends();
                succ_node.mark_recognized_dead_end();
            }
        }
        if (!succ_node.is_dead_end()) {
            m_open_list.push(
                    std::pair<bool, int>(!preferred.contains(aops[i]), node.get_h()),
                    succ.get_id());
        } else if (!succ_node.is_recognized_dead_end()) {
            m_call_stack.back().succ_result = DFSResult::UNRECOGNIZED;
        } else if (c_compute_recognized_neighbors) {
            m_recognized_neighbors.push_back(succ.get_id());
        }
    }
    preferred.clear();
    aops.clear();

    m_current_depth++;
    return true;
}

SearchStatus TarjanSearch::step()
{
    static StateSet recognized_neighbors;

    if (m_result == DFSResult::SOLVED) {
        return SearchStatus::SOLVED;
    }

    if (m_call_stack.empty()) {
        return SearchStatus::FAILED;
    }

    CallStackElement& elem = m_call_stack.back();

    bool in_dead_end_component = false;
    // if backtracked
    if (elem.last_successor_id != StateID::no_state) {
        State state = state_registry.lookup_state(elem.node.get_state_id());
        State succ = state_registry.lookup_state(elem.last_successor_id);
        SearchNode succ_node = m_search_space[succ];
        assert(succ_node.is_closed());
        elem.node.update_lowlink(succ_node.get_lowlink());
        elem.succ_result = std::max(elem.succ_result, m_result);
        if (succ_node.is_dead_end()) {
            if (m_result == DFSResult::DEAD_END_COMPONENT
                || (m_result == DFSResult::SCC_COMPLETED
                    && c_recompute_u
                    && evaluate_dead_end_heuristic(state))) {
                in_dead_end_component = true;
                elem.node.mark_recognized_dead_end();
                while (m_open_list.layer_size() > 0) {
                    StateID state_id = m_open_list.pop_minimum();
                    SearchNode node =
                        m_search_space[state_registry.lookup_state(state_id)];
                    if (!node.is_closed()) {
                        node.mark_recognized_dead_end();
                        statistics.inc_dead_ends();
                    } else if (node.is_onstack()) {
                        elem.node.update_lowlink(node.get_index());
                    } else {
                        assert(node.is_dead_end());
                        node.mark_recognized_dead_end();
                    }
                }
            } else if (c_compute_recognized_neighbors) {
                m_recognized_neighbors.push_back(succ.get_id());
            }
        }
    }

    bool fully_expanded = true;
    while (m_open_list.layer_size() > 0) {
        StateID succ_id = m_open_list.pop_minimum();
        State succ = state_registry.lookup_state(succ_id);
        SearchNode succ_node = m_search_space[succ];

        if (succ_node.is_dead_end()) {
            if (!succ_node.is_recognized_dead_end()) {
                elem.succ_result = std::max(elem.succ_result, DFSResult::UNRECOGNIZED);
            } else if (c_compute_recognized_neighbors) {
                m_recognized_neighbors.push_back(succ_id);
            }
        } else if (!succ_node.is_closed()) {
            if (task_properties::is_goal_state(task_proxy, succ)) {
                evaluate_dead_end_heuristic(succ); // mugs
                // m_pruning_method->prune_state(succ);
                std::vector<OperatorID> plan;
                m_search_space.trace_path(succ_node, plan);
                set_plan(plan);
                return SearchStatus::SOLVED;
            }
            if (expand(succ)) {
                elem.last_successor_id = succ_id;
                fully_expanded = false;
                break;
            } else if (!succ_node.is_recognized_dead_end()) {
                elem.succ_result = std::max(elem.succ_result, DFSResult::UNRECOGNIZED);
            } else if (c_compute_recognized_neighbors) {
                m_recognized_neighbors.push_back(succ_id);
            }
        } else if (succ_node.is_onstack()) {
            elem.node.update_lowlink(succ_node.get_index());
        } else {
            assert(false);
        }
    }

    m_result = DFSResult::FAILED;

    if (fully_expanded) {
        m_result = in_dead_end_component
                   ? DFSResult::DEAD_END_COMPONENT
                   : elem.succ_result;

        assert(elem.node.get_lowlink() <= elem.node.get_index());
        if (elem.node.get_index() == elem.node.get_lowlink()) {
            std::deque<State>::iterator it = m_stack.begin();
            std::deque<unsigned>::iterator rnoff_it = m_rn_offset.begin();
            unsigned scc_size = 0;
            while (true) {
                SearchNode snode = m_search_space[*it];
                snode.popped_from_stack();
                if (in_dead_end_component) {
                    if (!snode.is_dead_end()) {
                        /* m_progress.inc_partially_expanded_dead_ends(); */
                    }
                    snode.mark_recognized_dead_end();
                } else {
                    assert(!snode.is_dead_end());
                    snode.mark_dead_end();
                    /* m_progress.inc_expanded_dead_ends(); */
                }
                scc_size++;
                if ((it++)->get_id() == elem.node.get_state_id()) {
                    break;
                }
                if (c_compute_recognized_neighbors) {
                    rnoff_it++;
                }
            }
            bool entered_refinement = false;
            if (c_dead_end_refinement
                && !in_dead_end_component
                && (scc_size != m_stack.size() || c_refine_initial_state)
                && (elem.succ_result != DFSResult::UNRECOGNIZED || !c_compute_recognized_neighbors)) {
                entered_refinement = true;
                /* m_progress.inc_dead_end_refinements(); */
                if (c_compute_recognized_neighbors) {
                    std::deque<StateID>::iterator rnid_it =
                        m_recognized_neighbors.begin() + *rnoff_it;
                    while (rnid_it != m_recognized_neighbors.end()) {
                        recognized_neighbors.insert(state_registry.lookup_state(*rnid_it));
                        rnid_it++;
                    }
#if NDEBUG_VERIFY_RECOGNIZED_NEIGHBORS
                    {
                        std::vector<OperatorID> aops;
                        StateSet component;
                        std::deque<State>::iterator compit = m_stack.begin();
                        while (compit != it) {
                            component.insert(*compit);
                            compit++;
                        }
                        compit = m_stack.begin();
                        while (compit != it) {
                            g_successor_generator->generate_applicable_ops(*compit, aops);
                            for (unsigned i = 0; i < aops.size(); i++) {
                                State succ = state_registry.get_successor_state(
                                        *compit,
                                        task_proxy.get_operators()[aops[i]]);
                                if (!component.count(succ) && !recognized_neighbors.count(succ)) {
                                    SearchNode succnode = m_search_space[succ];
                                    std::cout << "STATE NOT FOUND! " << succ.get_id()
                                            << ": " << succnode.is_new()
                                            << "|" << succnode.is_closed()
                                            << "|" << succnode.is_dead_end()
                                            << "|" << succnode.is_onstack()
                                            << " " << succnode.get_index()
                                            << " " << succnode.get_lowlink()
                                            << std::endl;
                                }
                                assert(component.count(succ) || recognized_neighbors.count(succ));
                                assert(!component.count(succ) || !recognized_neighbors.count(succ));
                                assert(!recognized_neighbors.count(succ) || m_search_space[succ].is_dead_end());
                                assert(!recognized_neighbors.count(succ) || m_search_space[succ].is_recognized_dead_end());
                                assert(!recognized_neighbors.count(succ) ||  evaluate_dead_end_heuristic(succ));
                            }
                            compit++;
                            aops.clear();
                        }
                    }
#endif
                }
                // std::cout << "NEW DEAD END COMPONENT: " << std::flush;
                // for (auto cit = m_stack.begin(); cit != it; cit++) {
                //     std::cout << cit->get_id() << " " << std::flush;
                // }
                // std::cout << std::endl << "States on stack: " << m_stack.size() << std::endl;
            

                c_dead_end_refinement = m_learner->notify_dead_end_component(
                        StateComponentIterator<std::deque<State>::iterator>(m_stack.begin(), it),
                        StateComponentIterator<StateSet::iterator>(recognized_neighbors.begin(), recognized_neighbors.end()));
                recognized_neighbors.clear();
                if (!c_dead_end_refinement && c_compute_recognized_neighbors) {
                    c_compute_recognized_neighbors = false;
                    m_recognized_neighbors.clear();
                    m_rn_offset.clear();
                }
                if (c_dead_end_refinement) {
                    for (auto compit = m_stack.begin(); compit != it; compit++) {
#if NDEBUG_VERIFY_REFINEMENT
                        assert(evaluate_dead_end_heuristic(*compit));
#endif
                        assert(m_search_space[*compit].is_dead_end());
                        m_search_space[*compit].mark_recognized_dead_end();
                    }
                }
            }
            m_stack.erase(m_stack.begin(), it);
            if (c_compute_recognized_neighbors) {
                m_recognized_neighbors.erase(m_recognized_neighbors.begin() + *rnoff_it,
                                             m_recognized_neighbors.end());
                m_rn_offset.erase(m_rn_offset.begin(), ++rnoff_it);
            }
            m_result = (in_dead_end_component || (entered_refinement && c_dead_end_refinement))
                ? DFSResult::SCC_COMPLETED
                : DFSResult::UNRECOGNIZED;
        }
        m_call_stack.pop_back();
        m_current_depth--;
        m_open_list.pop_layer();
    }

    return SearchStatus::IN_PROGRESS;
}

void TarjanSearch::print_statistics() const
{
    std::cout << "Registered: " << state_registry.size() << " state(s)" << std::endl;
    statistics.print_detailed_statistics();
    if (m_learner != nullptr) {
        m_learner->print_statistics();
    }
    if (m_dead_end_identifier != nullptr) {
        // m_dead_end_identifier->print_evaluator_statistics();
    }
    m_pruning_method->print_statistics();
}

double
TarjanSearch::get_heuristic_refinement_time() const
{
    return m_learner != nullptr ? m_learner->get_refinement_timer()() : 0;
}

void TarjanSearch::add_options_to_parser(options::OptionParser &parser)
{
    SearchEngine::add_options_to_parser(parser);
    parser.add_option<std::shared_ptr<Evaluator>>("eval", "", options::OptionParser::NONE);
    parser.add_option<std::shared_ptr<Evaluator>>("u_eval", "", options::OptionParser::NONE);
    parser.add_option<std::shared_ptr<Evaluator>>("preferred", "", options::OptionParser::NONE);
    parser.add_option<std::shared_ptr<ConflictLearner>>("learn", "", options::OptionParser::NONE);
    parser.add_option<bool>("recompute_u",
                            "recompute dead-end detection heuristic after each refinement",
                            "true");
    parser.add_option<bool>("refine_initial_state", "", "false");
    parser.add_option<bool>("prune_eval_dead_ends", "", "true");
    parser.add_option<bool>("compatible_pruning_method", "", "false");
    parser.add_option<std::shared_ptr<PruningMethod>>(
        "pruning",
        "Pruning methods can prune or reorder the set of applicable operators in "
        "each state and thereby influence the number and order of successor states "
        "that are considered.",
        "null()");
}

}
}

static std::shared_ptr<SearchEngine>
_parse(options::OptionParser& parser)
{
    conflict_driven_learning::tarjan_search::TarjanSearch::add_options_to_parser(parser);
    options::Options opts = parser.parse();
    if (!parser.dry_run()) {
        return std::make_shared<conflict_driven_learning::tarjan_search::TarjanSearch>(opts);
    }
    return nullptr;
}

static Plugin<SearchEngine> _plugin("dfs", _parse);
