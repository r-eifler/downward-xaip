
#include "dfs_search.h"

#include "../../conflict_driven_learning/state_component.h"

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

struct StateLess {
  bool operator()(const State& s1, const State& s2) const {
      StateID i1 = s1.get_id();
      StateID i2 = s2.get_id();
      return i1(i1) < i2(i2);
  }
};

using StateSet = std::set<State, StateLess>;

namespace dfs_search
{

    DepthFirstSearchSearch::CallStackElement::CallStackElement(
            conflict_driven_learning::tarjan_search::SearchNode &node) :
    node(node), last_successor_id(StateID::no_state), succ_result(DFSResult::FAILED)
    {}

    DepthFirstSearchSearch::DepthFirstSearchSearch(const options::Options &opts)
    : SearchEngine(opts)
    , m_guidance(opts.contains("eval") ? opts.get<std::shared_ptr<Evaluator>>("eval") : nullptr)
    , m_dead_end_identifier(opts.contains("u_eval") ? opts.get<std::shared_ptr<Evaluator>>("u_eval") : nullptr)
    , m_pruning_method(opts.get<std::shared_ptr<PruningMethod>>("pruning"))
    , m_search_space(&state_registry)
    , m_current_index(0)
    , m_result(DFSResult::FAILED)
    , m_open_states(1)
    {}

void DepthFirstSearchSearch::initialize()
{
    std::cout << "Initializing depth first search ..." << std::endl;

    m_open_states = 1;
    m_current_index = 0;
    m_current_depth = 0;
    m_result = DFSResult::FAILED;

    State istate = state_registry.get_initial_state();

    conflict_driven_learning::tarjan_search::SearchNode inode = m_search_space[istate];
    m_pruning_method->initialize(task);
    if (!evaluate(istate)) {
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

bool DepthFirstSearchSearch::evaluate(const State& state)
{
    statistics.inc_evaluated_states();
    bool res = evaluate(state, m_guidance);
//    if (!res && !c_prune_eval_dead_ends) {
//        m_eval_result.set_evaluator_value(0);
//        return true;
//    }
    return res;
}

bool DepthFirstSearchSearch::evaluate(const State& state, std::shared_ptr<Evaluator> eval)
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

bool DepthFirstSearchSearch::evaluate_dead_end_heuristic(const State& state)
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

int DepthFirstSearchSearch::get_h_value() const
{
    return m_eval_result.get_evaluator_value();
}

bool DepthFirstSearchSearch::expand(const State& state)
{
    std::cout << " expand" << std::endl;
    std::cout << "----- Expand: " << state.get_id() << "-----" << std::endl;
    static std::vector<OperatorID> aops;

    conflict_driven_learning::tarjan_search::SearchNode node = m_search_space[state];
    m_open_states--;

    if (evaluate_dead_end_heuristic(state)) {
        return false;
    }

    //TODO pass remaining cost correctly
    // if (m_pruning_method->prune_state(state, INT32_MAX)) {
    //     node.mark_dead_end();
    //     return false;
    // }

    statistics.inc_expanded();

    node.close(m_current_index);
    m_current_index++;
    m_stack.push_front(state);
    m_call_stack.emplace_back(node);

    m_open_list.push_layer();

    successor_generator.generate_applicable_ops(state, aops);

    m_pruning_method->prune_operators(state, aops);

    statistics.inc_generated(aops.size());

    for (unsigned i = 0; i < aops.size(); i++) {
        State succ = state_registry.get_successor_state(state,task_proxy.get_operators()[aops[i]]);

        conflict_driven_learning::tarjan_search::SearchNode succ_node = m_search_space[succ];
        assert(succ_node.is_new() || succ_node.get_lowlink() <= succ_node.get_index());

        if (succ_node.is_new()) {
            if (evaluate(succ)) {
                succ_node.open(node, aops[i], get_h_value());
                m_open_states++;
            } else {
              statistics.inc_dead_ends();
            }
        }
        if (!succ_node.is_dead_end()) {
            m_open_list.push(node.get_h(),succ.get_id());
        }
    }

    aops.clear();

    m_current_depth++;
    return true;
}

SearchStatus DepthFirstSearchSearch::step()
{

    if (m_result == DFSResult::SOLVED) {
        // this->run_finished_successfully = true;
        return SearchStatus::SOLVED;
    }

    if (m_call_stack.empty()) {
        // this->run_finished_successfully = true;
        return SearchStatus::FAILED;
    }

    CallStackElement& elem = m_call_stack.back();

    // if backtracked
    if (elem.last_successor_id != StateID::no_state) {

        State state = state_registry.lookup_state(elem.node.get_state_id());
        State succ = state_registry.lookup_state(elem.last_successor_id);
        conflict_driven_learning::tarjan_search::SearchNode succ_node = m_search_space[succ];
        assert(succ_node.is_closed());

        elem.node.update_lowlink(succ_node.get_lowlink());
        elem.succ_result = std::max(elem.succ_result, m_result);
    }

    bool fully_expanded = true;

    while (m_open_list.layer_size() > 0) {
        StateID succ_id = m_open_list.pop_minimum();
        std::cout << succ_id << ": ";
        State succ = state_registry.lookup_state(succ_id);
        conflict_driven_learning::tarjan_search::SearchNode succ_node = m_search_space[succ];

        if (succ_node.is_dead_end()) {
            continue; //Pruning can only get stronger so once a dead end always a dead end
        } else if (!succ_node.is_closed()) {
            if (task_properties::is_goal_state(task_proxy, succ)) {
                evaluate_dead_end_heuristic(succ); // mugs
                std::vector<OperatorID> plan;
                m_search_space.trace_path(succ_node, plan);
                set_plan(plan);
                return SearchStatus::SOLVED;
            }
            if (expand(succ)) {
                elem.last_successor_id = succ_id;
                fully_expanded = false;
                break;
            }
        } else if (succ_node.is_onstack()) {
            elem.node.update_lowlink(succ_node.get_index());
        } else {
            assert(false);
        }
    }

    m_result = DFSResult::FAILED;

    //backtrack
    if (fully_expanded) { 
        m_result = elem.succ_result;

        assert(elem.node.get_lowlink() <= elem.node.get_index()); 
        if (elem.node.get_index() == elem.node.get_lowlink()) {
            std::deque<State>::iterator it = m_stack.begin();

            while (true) {
                conflict_driven_learning::tarjan_search::SearchNode snode = m_search_space[*it];
                snode.popped_from_stack();
                if ((it++)->get_id() == elem.node.get_state_id()) {
                    break;
                }
            }

            m_stack.erase(m_stack.begin(), it);

        }
        m_call_stack.pop_back();
        m_current_depth--;
        m_open_list.pop_layer();
    }

    return SearchStatus::IN_PROGRESS;
}

void DepthFirstSearchSearch::print_statistics() const
{
    // SearchEngine::print_statistics(); // TODO is there something missing?
    std::cout << "Registered: " << state_registry.size() << " state(s)" << std::endl;
    statistics.print_detailed_statistics();

    if (m_dead_end_identifier != nullptr) {
        // m_dead_end_identifier->print_statistics();
    }
    m_pruning_method->print_statistics();
}

void DepthFirstSearchSearch::add_options_to_parser(options::OptionParser &parser)
{
    SearchEngine::add_options_to_parser(parser);
    parser.add_option<std::shared_ptr<Evaluator>>("eval", "", options::OptionParser::NONE);
    parser.add_option<std::shared_ptr<Evaluator>>("u_eval", "", options::OptionParser::NONE);
    parser.add_option<bool>("prune_eval_dead_ends", "", "true");
    parser.add_option<std::shared_ptr<PruningMethod>>(
        "pruning",
        "Pruning methods can prune or reorder the set of applicable operators in "
        "each state and thereby influence the number and order of successor states "
        "that are considered.",
        "null()");
}


}

static std::shared_ptr<SearchEngine> _parse(options::OptionParser& parser)
{

    dfs_search::DepthFirstSearchSearch::add_options_to_parser(parser);
    options::Options opts = parser.parse();
    if (!parser.dry_run()) {
        return std::make_shared<dfs_search::DepthFirstSearchSearch>(opts);
    }
    return nullptr;
}

static Plugin<SearchEngine> _plugin("simple_dfs", _parse);

