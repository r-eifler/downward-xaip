#include "bounded_cost_tarjan_search.h"

#include "../algorithms/ordered_set.h"
#include "../evaluation_context.h"
#include "../evaluation_result.h"
#include "../operator_cost.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../pruning_method.h"
#include "../state_registry.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../tasks/cost_adapted_task.h"
#include "../utils/system.h"
#include "../state_id.h"
#include "hc_heuristic.h"
#include "state_component.h"

#include <algorithm>
#include <limits>
#include <unordered_map>

#ifndef NDEBUG
#define DEBUG_BOUNDED_COST_DFS_ASSERT_LEARNING 1
#define DEBUG_BOUNDED_COST_DFS_ASSERT_NEIGHBORS 1
#endif

namespace {
struct StateIDHash {
    std::size_t operator()(const StateID& state_id) const {
	return std::hash<std::size_t>()(state_id(state_id));
    }
};
}

namespace conflict_driven_learning {
namespace bounded_cost {

static const int INF = std::numeric_limits<int>::max();
static const int UNDEFINED = (INF - 1);

static int
_get_bound(const int& status)
{
    return status & INF;
}

static void
_set_bound(int& status, int bound)
{
    assert(bound >= 0 && bound != UNDEFINED);
    status = bound;
    assert(_get_bound(status) == bound);
}

BoundedCostTarjanSearch::Locals::Locals(
    const State& state,
    bool zero_layer,
    unsigned size)
    : state(state)
    , successor_op(OperatorID::no_operator)
    , zero_layer(zero_layer)
    , neighbors_size(size)
{
}

BoundedCostTarjanSearch::ExpansionInfo::ExpansionInfo()
    : index(INF)
    , lowlink(INF)
{
}

BoundedCostTarjanSearch::ExpansionInfo&
    BoundedCostTarjanSearch::HashMap::operator[](const StateID& state)
{
    return data[state];
}

void
BoundedCostTarjanSearch::HashMap::remove(const StateID& state)
{
    data.erase(data.find(state));
}

BoundedCostTarjanSearch::PerLayerData::PerLayerData()
    : index(0)
{
}

BoundedCostTarjanSearch::BoundedCostTarjanSearch(const options::Options& opts)
    : SearchEngine(opts)
    , c_ignore_eval_dead_ends(opts.get<bool>("ignore_eval_dead_ends"))
    , c_refinement_toggle(false)
    , c_compute_neighbors(false)
    , c_make_neighbors_unique(opts.get<bool>("unique_neighbors"))
    , c_max_bound(
          opts.contains("max_bound") ? opts.get<int>("max_bound") : bound)
    , c_bound_step(opts.get<double>("step"))
    // , c_learning_belt(opts.contains("learning_belt") ? bound -
    // opts.get<int>("learning_belt") : 0)
    , m_task(
          cost_type != OperatorCost::NORMAL
              ? std::make_shared<tasks::CostAdaptedTask>(task, cost_type)
              : task)
    , m_task_proxy(*m_task)
    , m_expansion_evaluator(
          opts.contains("eval") ? opts.get<std::shared_ptr<Evaluator>>("eval") : nullptr)
    , m_preferred(
          opts.contains("preferred") ? opts.get<std::shared_ptr<Evaluator>>("preferred")
                                     : nullptr)
    , m_pruning_evaluator(
          opts.contains("u_eval") ? opts.get<std::shared_ptr<Evaluator>>("u_eval") : nullptr)
    , m_refiner(
          opts.contains("learn")
              ? opts.get<std::shared_ptr<HeuristicRefiner>>("learn")
              : nullptr)
    , m_pruning_method(opts.get<std::shared_ptr<PruningMethod>>("pruning"))
    , m_state_information(UNDEFINED)
    , m_solved(false)
    , m_last_state(StateID::no_state)
{
    if (m_refiner != nullptr) {
        c_refinement_toggle = true;
        c_compute_neighbors = m_refiner->requires_neighbors();
        // m_pruning_evaluator = m_refiner->get_underlying_heuristic().get();
    }
    if (m_expansion_evaluator) {
        m_expansion_evaluator->get_path_dependent_evaluators(
            m_path_dependent_evaluators);
    }
    if (m_preferred) {
        m_preferred->get_path_dependent_evaluators(m_path_dependent_evaluators);
    }
    if (m_pruning_evaluator) {
        m_pruning_evaluator->get_path_dependent_evaluators(
            m_path_dependent_evaluators);
    }

    // if (c_max_bound == std::numeric_limits<int>::max()) {
    //     std::cerr << "bounded cost depth first search requires bound <
    //     infinity"
    //                 << std::endl;
    //     utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
    // }
    m_current_g = 0;
}

#if 0
bool
BoundedCostTarjanSearch::increment_bound_and_push_initial_state()
{
    m_current_g = 0;
    State istate = state_registry.get_initial_state();
    while (true) {
        assert(bound <= _get_bound(m_state_information[istate]));
        bound = _get_bound(m_state_information[istate]);
        if (bound >= c_max_bound || bound == INF) {
            break;
        }
        bound = std::min(c_max_bound, (int)(c_bound_step * bound));
        if (expand(istate)) {
            std::cout << "incremented bound to " << bound << std::endl;
            return true;
        }
    }
    return false;
}
#endif

void
BoundedCostTarjanSearch::initialize()
{
    State istate = state_registry.get_initial_state();
    for (Evaluator* pde : m_path_dependent_evaluators) {
        pde->notify_initial_state(istate);
    }
    _set_bound(m_state_information[istate], 0);
    if (task_properties::is_goal_state(m_task_proxy, istate)) {
        if (log.is_at_least_normal())
            log << "Initial state satisfies goal condition!" << std::endl;
        m_solved = true;
    } else {
        m_pruning_method->initialize(task);
        if ((!c_ignore_eval_dead_ends
             && !evaluate(istate, m_expansion_evaluator.get(), 0))
            || !expand(istate)) {
            // || !increment_bound_and_push_initial_state()) {
            std::cout << "Initial state is dead-end!" << std::endl;
        }
    }
}

bool
BoundedCostTarjanSearch::evaluate(
    const State& state,
    Evaluator* eval,
    int g)
{
    if (eval == NULL) {
        m_eval_result.set_evaluator_value(0);
        return true;
    }
    EvaluationContext ctxt(state, g, false, nullptr);
    m_eval_result = eval->compute_result(ctxt);
    if (m_eval_result.is_infinite()) {
        return false;
    }
    return true;
}

bool
BoundedCostTarjanSearch::expand(const State& state)
{
    return expand(state, NULL);
}

bool
BoundedCostTarjanSearch::expand(const State& state, PerLayerData* layer)
{
    static std::vector<OperatorID> aops;
    assert(aops.empty());
    static ordered_set::OrderedSet<OperatorID> preferred;
    assert(preferred.empty());

    int& status = m_state_information[state];
    assert(
        _get_bound(status) != INF && m_current_g + _get_bound(status) < bound);

    // TODO check if already computed after last refinement, and if so skip
    // evaluation
    if (!evaluate(state, m_pruning_evaluator.get(), m_current_g)) {
        _set_bound(status, INF);
        // std::cout << "Dead end -> " << _get_bound(status) << std::endl;
        return false;
    }

    if (m_eval_result.get_evaluator_value() > _get_bound(status)) {
        _set_bound(status, m_eval_result.get_evaluator_value());
        if (m_current_g + m_eval_result.get_evaluator_value() >= bound) {
            // std::cout << "Exceeds bound-> " << m_current_g << " + " <<
            // _get_bound(status) << " > " << bound << std::endl;
            return false;
        }
    }

    statistics.inc_expanded();

    bool has_zero_cost = layer != NULL;
    m_call_stack.emplace_back(state, has_zero_cost, m_neighbors.size());
    Locals& locals = m_call_stack.back();

    successor_generator.generate_applicable_ops(state, aops);
    m_pruning_method->prune_operators(state, aops);
    statistics.inc_generated(aops.size());
    if (m_preferred) {
        if (evaluate(state, m_preferred.get(), m_current_g)) {
            const std::vector<OperatorID>& pref =
                m_eval_result.get_preferred_operators();
            for (int i = pref.size() - 1; i >= 0; i--) {
                preferred.insert(pref[i]);
            }
        }
    }
    for (unsigned i = 0; i < aops.size(); i++) {
        auto op = m_task_proxy.get_operators()[aops[i]];
        State succ = state_registry.get_successor_state(state, op);
        int& succ_info = m_state_information[succ];
        for (Evaluator* pde : m_path_dependent_evaluators) {
            pde->notify_state_transition(state, aops[i], succ);
        }
        if (_get_bound(succ_info) != INF
            && (evaluate(
                    succ, m_expansion_evaluator.get(), m_current_g + op.get_cost())
                || c_ignore_eval_dead_ends)) {
            has_zero_cost = has_zero_cost || op.get_cost() == 0;
            std::pair<bool, int> key(
                !preferred.contains(aops[i]),
                m_eval_result.get_evaluator_value());
            locals.open[key].emplace_back(aops[i], succ.get_id());
        } else if (c_compute_neighbors) {
            _set_bound(succ_info, INF);
            m_neighbors.emplace_back(
                m_task->get_operator_cost(aops[i].get_index(), false), succ);
        }
    }
    preferred.clear();
    aops.clear();

    if (has_zero_cost && layer == NULL) {
        m_layers.emplace_back();
        m_last_layer = layer = &m_layers.back();
        locals.zero_layer = true;
    }

    if (layer != NULL) {
        ExpansionInfo& state_info = layer->state_infos[state.get_id()];
        state_info.index = state_info.lowlink = layer->index++;
        layer->stack.push_front(state);
    }

    return true;
}

SearchStatus
BoundedCostTarjanSearch::step()
{
    static std::vector<std::pair<int, State>> component_neighbors;
    static std::unordered_map<StateID, int, StateIDHash> hashed_neighbors;

    if (m_solved) {
        Plan plan;
        while (!m_call_stack.empty()) {
            plan.push_back(m_call_stack.back().successor_op);
            m_call_stack.pop_back();
        }
        std::reverse(plan.begin(), plan.end());
        set_plan(plan);
        return SearchStatus::SOLVED;
    }

    if (m_call_stack.empty()) {
#if 0
        if (increment_bound_and_push_initial_state()) {
            return SearchStatus::IN_PROGRESS;
        }
#endif
        if (log.is_at_least_normal())
            log << "Completely explored state space" << std::endl;
        return SearchStatus::FAILED;
    }

    assert(bound < INF);

    Locals& locals = m_call_stack.back();
    ExpansionInfo* state_info = NULL;
    if (locals.zero_layer) {
        state_info = &m_last_layer->state_infos[locals.state.get_id()];
        assert(state_info->index < INF && state_info->lowlink < INF);
    }

    if (locals.successor_op != OperatorID::no_operator) {
        assert(m_last_state != StateID::no_state);
        if (m_last_state_lowlink != INF) {
            assert(state_info != NULL);
            assert(
                m_task->get_operator_cost(
                    locals.successor_op.get_index(), false)
                == 0);
            state_info->lowlink =
                std::min(state_info->lowlink, m_last_state_lowlink);
            // check if a) sth has been learned and b) want to reevaluate h
        } else {
            int cost = m_task->get_operator_cost(
                locals.successor_op.get_index(), false);
            m_current_g -= cost;
            if (c_compute_neighbors) {
                m_neighbors.emplace_back(
                    cost, state_registry.lookup_state(m_last_state));
            }
        }
    }

    bool all_children_explored = true;
    while (!locals.open.empty()) {
        auto it = locals.open.begin();
        auto succ = it->second.back();
        it->second.pop_back();
        if (it->second.empty()) {
            locals.open.erase(it);
        }
        locals.successor_op = succ.first;
        State succ_state = state_registry.lookup_state(succ.second);
        int cost =
            m_task->get_operator_cost(locals.successor_op.get_index(), false);
        m_current_g += cost;
        if (m_current_g < bound) {
            assert(cost > 0 || locals.zero_layer);
            int& succ_status = m_state_information[succ_state];
            if (succ_status == UNDEFINED) {
                succ_status = 0;
                if (task_properties::is_goal_state(m_task_proxy, succ_state)) {
                    evaluate(
                        succ_state, m_expansion_evaluator.get(), m_current_g); // mugs
                    evaluate(
                        succ_state, m_pruning_evaluator.get(), m_current_g); // mugs
                    ///TODO do we need the prune state function ?
                    // m_pruning_method->prune_state(succ_state);
                    m_solved = true;
                    m_current_g -= cost;
                    return SearchStatus::IN_PROGRESS;
                }
            }
            bool dead = false;
            int succ_bound = _get_bound(succ_status);
            if (succ_bound != INF && m_current_g + succ_bound < bound) {
                if (cost == 0) {
                    assert(state_info != NULL);
                    ExpansionInfo& succ_info =
                        m_last_layer->state_infos[succ.second];
                    if (succ_info.index == INF) {
                        if (expand(succ_state, m_last_layer)) {
                            all_children_explored = false;
                            break;
                        } else {
                            dead = true;
                            m_last_layer->state_infos.remove(succ.second);
                            assert(
                                _get_bound(succ_status) == INF
                                || m_current_g + _get_bound(succ_status)
                                    >= bound);
                        }
                    } else {
                        // onstack
                        state_info->lowlink =
                            std::min(state_info->lowlink, succ_info.index);
                    }
                } else {
                    if (expand(succ_state, NULL)) {
                        all_children_explored = false;
                        break;
                    } else {
                        dead = true;
                    }
                }
            } else {
                dead = true;
            }
            if (dead && c_compute_neighbors) {
                m_neighbors.emplace_back(cost, succ_state);
            }
        } else if (c_compute_neighbors) {
            // neighbor should not be required
            m_neighbors.emplace_back(cost, succ_state);
        }
        m_current_g -= cost;
    }

    if (all_children_explored) {
        if (state_info == NULL || state_info->index == state_info->lowlink) {
            std::unique_ptr<SuccessorComponent> neighbors = nullptr;
            if (c_refinement_toggle) { // && c_learning_belt <= m_current_g) {
                if (c_compute_neighbors && c_make_neighbors_unique) {
                    assert(
                        component_neighbors.empty()
                        && hashed_neighbors.empty());
                    component_neighbors.reserve(
                        m_neighbors.size() - locals.neighbors_size);
                    hashed_neighbors.reserve(
                        m_neighbors.size() - locals.neighbors_size);
                    auto it = m_neighbors.rbegin();
                    for (unsigned size = m_neighbors.size();
                         size > locals.neighbors_size;
                         size--) {
                        assert(it != m_neighbors.rend());
                        auto insrted =
                            hashed_neighbors.insert(std::pair<StateID, int>(
                                it->second.get_id(),
                                component_neighbors.size()));
                        if (insrted.second) {
                            component_neighbors.push_back(*it);
                        } else {
                            component_neighbors[insrted.first->second].first =
                                std::min(
                                    component_neighbors[insrted.first->second]
                                        .first,
                                    it->first);
                        }
                        it++;
                    }
                    hashed_neighbors.clear();
                    neighbors = std::unique_ptr<SuccessorComponent>(
                        new SuccessorComponentIterator<
                            std::vector<std::pair<int, State>>::iterator>(
                            component_neighbors.begin(),
                            component_neighbors.end()));
                } else {
                    neighbors = std::unique_ptr<SuccessorComponent>(
                        new SuccessorComponentIterator<
                            std::deque<std::pair<int, State>>::iterator>(
                            m_neighbors.begin() + locals.neighbors_size,
                            m_neighbors.end()));
                }
#if DEBUG_BOUNDED_COST_DFS_ASSERT_NEIGHBORS
                if (c_compute_neighbors) {
                    std::unordered_set<StateID, StateIDHash> component_state_ids;
                    if (state_info != NULL) {
                        for (auto it = m_last_layer->stack.begin();; it++) {
                            component_state_ids.insert((*it).get_id());
                            if ((*it).get_id()
                                == locals.state.get_id()) {
                                break;
                            }
                        }
                    } else {
                        component_state_ids.insert(locals.state.get_id());
                    }
                    std::unordered_set<StateID, StateIDHash> successor_state_ids;
                    for (auto it = m_neighbors.begin() + locals.neighbors_size;
                         it != m_neighbors.end();
                         it++) {
                        successor_state_ids.insert(it->second.get_id());
                    }
                    for (auto it = component_state_ids.begin();
                         it != component_state_ids.end();
                         it++) {
                        State state = state_registry.lookup_state(*it);
                        std::vector<OperatorID> aops;
                        successor_generator.generate_applicable_ops(
                            state, aops);
                        for (int i = aops.size() - 1; i >= 0; i--) {
                            OperatorProxy op =
                                m_task_proxy.get_operators()[aops[i]];
                            State succ =
                                state_registry.get_successor_state(state, op);
                            assert(
                                component_state_ids.count(succ.get_id())
                                || successor_state_ids.count(succ.get_id()));
                            int status = m_state_information[succ];
                            if (!component_state_ids.count(succ.get_id())) {
#if 0
                                if (_get_bound(status) != INF && _get_bound(status) + m_current_g + op.get_cost() < bound) {
                                    std::cout << "rn property is violated" << std::endl;
                                    std::cout << "cost_bound = " << bound << std::endl;
                                    std::cout << "current g = " << m_current_g << std::endl;
                                    std::cout << "op cost = " << op.get_cost() << std::endl;
                                    std::cout << "successor bound = " << _get_bound(status) << std::endl;
                                }
#endif
                                assert(
                                    m_current_g + op.get_cost() >= bound
                                    || _get_bound(status) == INF
                                    || _get_bound(status) + m_current_g
                                            + op.get_cost()
                                        >= bound);
                            }
                        }
                    }
                }
#endif
            }
            if (state_info == NULL) {
                _set_bound(
                    m_state_information[locals.state], bound - m_current_g);
                if (c_refinement_toggle) { // && c_learning_belt <= m_current_g)
                                           // {
                    SingletonComponent<State> component(locals.state);
                    c_refinement_toggle = m_refiner->notify(
                        bound - m_current_g, component, *neighbors);
                }
            } else {
                assert(m_last_layer != NULL);
                auto component_end = m_last_layer->stack.begin();
                while (true) {
                    _set_bound(
                        m_state_information[*component_end],
                        bound - m_current_g);
                    m_last_layer->state_infos.remove(component_end->get_id());
                    if ((component_end++)->get_id()
                        == locals.state.get_id()) {
                        break;
                    }
                }
                if (c_refinement_toggle) { // && c_learning_belt <= m_current_g)
                                           // {
                    StateComponentIterator<std::deque<State>::iterator>
                        component(m_last_layer->stack.begin(), component_end);
                    c_refinement_toggle = m_refiner->notify(
                        bound - m_current_g, component, *neighbors);
                }
                m_last_layer->stack.erase(
                    m_last_layer->stack.begin(), component_end);
                if (m_last_layer->stack.empty()) {
                    // layer completely explored
                    m_layers.pop_back();
                    m_last_layer = m_layers.empty() ? NULL : &m_layers.back();
                }
            }
            // #if DEBUG_BOUNDED_COST_DFS_ASSERT_LEARNING
            //             if (c_refinement_toggle) {
            //                 bool dead = !evaluate(locals.state,
            //                 m_pruning_evaluator);
            //                 // std::cout << "refinement result => dead=" <<
            //                 dead << " h=" <<
            //                 m_eval_result.get_evaluator_value() << std::endl;
            //                 assert(dead || m_current_g +
            //                 m_eval_result.get_evaluator_value() >= bound);
            //             }
            // #endif
            component_neighbors.clear();
            if (locals.neighbors_size != m_neighbors.size()) {
                m_neighbors.erase(
                    m_neighbors.begin() + locals.neighbors_size,
                    m_neighbors.end());
            }
            assert(m_neighbors.size() == locals.neighbors_size);
            m_last_state_lowlink = INF;
        } else {
            m_last_state_lowlink = state_info->lowlink;
        }
        m_last_state = locals.state.get_id();
        m_call_stack.pop_back();
    }

    return SearchStatus::IN_PROGRESS;
}

void
BoundedCostTarjanSearch::print_statistics() const
{
    std::cout << "Registered: " << state_registry.size() << " state(s)"
              << std::endl;
    statistics.print_detailed_statistics();
    if (m_refiner != nullptr) {
        m_refiner->print_statistics();
    }
    if (m_expansion_evaluator != nullptr) {
        m_expansion_evaluator->print_statistics();
    }
    if (m_pruning_evaluator != nullptr) {
        m_pruning_evaluator->print_statistics();
    }
#ifndef NDEBUG
    hc_heuristic::HCHeuristic* h =
        dynamic_cast<hc_heuristic::HCHeuristic*>(m_pruning_evaluator);
    if (h != NULL) {
        h->dump_conjunctions();
    }
#endif
    m_pruning_method->print_statistics();
}

// double
// BoundedCostTarjanSearch::get_heuristic_refinement_time() const
// {
//     return m_refiner != nullptr ? m_refiner->get_refinement_timer()() : 0;
// }

void
BoundedCostTarjanSearch::add_options_to_parser(options::OptionParser& parser)
{
    parser.add_option<std::shared_ptr<Evaluator>>("eval", "", options::OptionParser::NONE);
    parser.add_option<std::shared_ptr<Evaluator>>("u_eval", "", options::OptionParser::NONE);
    parser.add_option<std::shared_ptr<HeuristicRefiner>>(
        "learn", "", options::OptionParser::NONE);
    parser.add_option<bool>("ignore_eval_dead_ends", "", "false");
    parser.add_option<bool>("unique_neighbors", "", "true");
    parser.add_option<int>("max_bound", "", options::OptionParser::NONE);
    parser.add_option<double>("step", "", "2.0");
    // parser.add_option<int>("learning_belt", "", options::OptionParser::NONE);
    parser.add_option<std::shared_ptr<Evaluator>>("preferred", "", options::OptionParser::NONE);
    parser.add_option<std::shared_ptr<PruningMethod>>(
        "pruning",
        "Pruning methods can prune or reorder the set of applicable operators "
        "in "
        "each state and thereby influence the number and order of successor "
        "states "
        "that are considered.",
        "null()");
    SearchEngine::add_options_to_parser(parser);
}

} // namespace bounded_cost
} // namespace conflict_driven_learning

static std::shared_ptr<SearchEngine>
_parse(options::OptionParser& p)
{
    conflict_driven_learning::bounded_cost::BoundedCostTarjanSearch::
        add_options_to_parser(p);
    options::Options opts = p.parse();
    if (!p.dry_run()) {
        return std::make_shared<
            conflict_driven_learning::bounded_cost::BoundedCostTarjanSearch>(
            opts);
    }
    return nullptr;
}

static Plugin<SearchEngine> _plugin("bounded_cost_dfs", _parse);

