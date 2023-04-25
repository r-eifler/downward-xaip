#include "trap_neighbors_refinement.h"
#include "trap_unsat_heuristic.h"
#include "strips_compilation.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../task_proxy.h"
#include "../utils/hash.h"

#include <unordered_set>
#include <limits>
#include <memory>

namespace std {
template<>
struct hash<StateID> {
    size_t operator()(const StateID& state_id) const {
	return hash<size_t>()(state_id(state_id));
    }
};
}

namespace conflict_driven_learning {
namespace traps {

static const int INTMIN = std::numeric_limits<int>::min();

TrapNeighborsRefinement::TrapNeighborsRefinement(const options::Options& opts)
    : c_recompute_reachability(opts.get<bool>("recompute_reachability"))
    , m_trap(std::dynamic_pointer_cast<TrapUnsatHeuristic>(opts.get<std::shared_ptr<Evaluator>>("trap")))
    , m_task(m_trap->get_abstract_task().get())
{
    strips::initialize(*m_task);
    const strips::Task& task = strips::get_task();
    m_fact_mutex_with_goal.resize(strips::num_facts(), false);
    for (unsigned p = 0; p < m_fact_mutex_with_goal.size(); p++) {
        m_fact_mutex_with_goal[p] = task.are_mutex(p, task.get_goal());
    }
    m_variable_order.resize(m_task->get_num_variables());
    for (int i = 0; i < m_task->get_num_variables(); i++) {
        m_variable_order[i] = m_task->get_num_variables() - i - 1;
    }
}

void
TrapNeighborsRefinement::add_options_to_parser(options::OptionParser& parser)
{
    parser.add_option<bool>("recompute_reachability", "", "false");
    parser.add_option<std::shared_ptr<Evaluator>>("trap");
}

Evaluator*
TrapNeighborsRefinement::get_underlying_heuristic()
{
    return m_trap.get();
}

bool
TrapNeighborsRefinement::requires_recognized_neighbors() const
{
    return false;
}

bool
TrapNeighborsRefinement::learn_from_dead_end_component(
        StateComponent& component,
        StateComponent& )
{
    if (m_trap->evaluate_check_dead_end(component.current())) {
        return true;
    }

    CounterBasedFormula lookup;
    lookup.set_num_keys(strips::num_facts());
    std::vector<std::vector<unsigned> > conjunctions;
    std::vector<unsigned> num_facts_goal_mutex;
#ifndef NDEBUG
    std::unordered_set<StateID> state_ids;
#endif

    auto task = m_trap->get_abstract_task();
    while (!component.end()) {
        conjunctions.emplace_back(m_task->get_num_variables());
        std::vector<unsigned>& conj = conjunctions.back();
        conj.clear();
        const State& state = component.current();

        // the following code is required if goal in search is different from
        // goal in heuristic
        bool is_goal_state = true;
        for (int i = 0; i < task->get_num_goals(); i++) {
            FactPair g = task->get_goal_fact(i);
            if (state[g.var].get_value() != g.value) {
                is_goal_state = false;
                break;
            }
        }
        if (is_goal_state) {
            return false;
        }

        for (int var = 0; var < m_task->get_num_variables(); var++) {
            conj.push_back(strips::get_fact_id(var, state[var].get_value()));
        }
        lookup.insert(conj);
        unsigned num = 0;
        for (const unsigned &p : conj) {
            if (m_fact_mutex_with_goal[p]) {
                num++;
            }
        }
        num_facts_goal_mutex.push_back(num);
#ifndef NDEBUG
        assert(!state_ids.count(state.get_id()));
        state_ids.insert(state.get_id());
#endif
        component.next();
    }

    TrapUnsatHeuristic::Formula& formula = m_trap->get_all_conjunctions_formula();

    std::vector<std::vector<ForwardHyperTransition> > transitions(conjunctions.size());
    std::vector<std::vector<ForwardHyperTransition> > previous_transitions;
    std::vector<int> removed_fact(conjunctions.size(), -1);
    std::vector<int> duplicate(conjunctions.size(), 0);
    std::vector<int> var_indices(m_variable_order.size());
    for (unsigned var = 0; var < var_indices.size(); var++) {
        var_indices[var] = var;
    }
    
    int num_minimized = 0;

    for (unsigned i = 0; i < m_variable_order.size(); i++) {
        int var = m_variable_order[i];
        int var_idx = var_indices[var];
        bool satisfied = true;
        for (unsigned j = 0; j < conjunctions.size(); j++) {
            if (duplicate[j]) {
                continue;
            }
            std::vector<unsigned>& conj = conjunctions[j];
            assert((unsigned)var_idx < conj.size());
            removed_fact[j] = conj[var_idx];
            assert(strips::get_variable_assignment(removed_fact[j]).first == var);
            if (m_fact_mutex_with_goal[removed_fact[j]]
                    && --num_facts_goal_mutex[j] == 0) {
                satisfied = false;
                removed_fact[j] = -1;
                break;
            }
            conj.erase(conj.begin() + var_idx);
            duplicate[j] = lookup.contains_subset_of(conj) ? -1 : 0;
            lookup.delete_element(j, conj, removed_fact[j]); 
        }
        if (satisfied) {
            previous_transitions.swap(transitions);
            transitions.clear();
            transitions.resize(conjunctions.size());
            for (unsigned j = 0; j < conjunctions.size(); j++) {
                if (duplicate[j]) {
                    continue;
                }
                const std::vector<unsigned>& conj = conjunctions[j];
                std::vector<ForwardHyperTransition>& ts = transitions[j];
                bool is_dead = m_trap->are_dead_ends(conj);
                if (m_trap->for_every_progression_action(conj, [&](unsigned op) {
                    bool closed = is_dead;
                    ts.emplace_back(op);
                    ForwardHyperTransition& t = ts.back();
                    m_trap->progression(conj, op, t.progression);
                    formula.forall_subsets(t.progression, [&](unsigned id) {
                        t.destinations.push_back(id);
                        closed = closed || !m_trap->can_reach_goal(id);
                        return false;
                    });
                    lookup.forall_subsets(t.progression, [&](unsigned idx) {
                        if (!duplicate[idx]) {
                            closed = true;
                            t.destinations.push_back(INTMIN + idx);
                        }
                        return false;
                    });
                    if (!closed) {
                        closed = t.dead = m_trap->are_dead_ends(t.progression);
                    }
                    return !closed;
                })) {
                    satisfied = false;
                    transitions.swap(previous_transitions);
                    break;
                }
            }
        }
        if (satisfied) {
            for (unsigned var2 = var + 1; var2 < m_variable_order.size(); var2++) {
                var_indices[var2]--;
            }
            for (unsigned j = 0; j < duplicate.size(); j++) {
                if (duplicate[j] == -1) {
                    duplicate[j] = 1;
                }
            }
            num_minimized++;
        } else {
            for (unsigned j = 0; j < conjunctions.size(); j++) {
                if (duplicate[j] == 1) {
                    continue;
                }
                if (removed_fact[j] == -1) {
                    break;
                }
                duplicate[j] = 0;
                std::vector<unsigned>& conj = conjunctions[j];
                conj.insert(conj.begin() + var_idx, removed_fact[j]);
                lookup.insert_element(j, conj, removed_fact[j], 0);
                if (m_fact_mutex_with_goal[removed_fact[j]]) {
                    num_facts_goal_mutex[j]++;
                }
            }
        }
#ifndef NDEBUG
        for (unsigned j = 0; j < conjunctions.size(); j++) {
            assert(duplicate[j] == 0 || duplicate[j] == 1);
            assert(std::is_sorted(conjunctions[j].begin(), conjunctions[j].end()));
            assert(lookup.contains_subset_of(conjunctions[j]));
        }
#endif
    }

    if (num_minimized == 0) {
        return false;
    }

    int old_num_conjunctions = formula.size();
    for (unsigned i = 0; i < duplicate.size(); i++) {
        if (duplicate[i]) {
            duplicate[i] = -1;
            continue;
        }
        duplicate[i] = m_trap->insert_conjunction(conjunctions[i]).first;
    }

    for (unsigned i = 0; i < duplicate.size(); i++) {
        if (duplicate[i] == -1) {
            continue;
        }
        unsigned conj_id = duplicate[i];
        std::vector<ForwardHyperTransition>& ts = transitions[i];
        for (unsigned j = 0; j < ts.size(); j++) {
            ForwardHyperTransition& t = ts[j];
            for (int k = t.destinations.size() - 1; k >= 0; k--) {
                int ref = t.destinations[k];
                if (ref >= 0) {
                    break;
                }
                ref = ref - INTMIN;
                assert(ref < (int) duplicate.size());
                assert(duplicate[ref] >= 0);
                if (duplicate[ref] < old_num_conjunctions) {
                    t.destinations.erase(t.destinations.begin() + k);
                } else {
                    t.destinations[k] = duplicate[ref];
                }
            }
        }
        m_trap->set_transitions(conj_id, std::move(ts));
    }

#if 0
    ndef NDEBUG
    // unsigned component_size = 0;

    std::vector<unsigned> conjs;
    component.reset();
    while (!component.end()) {
        const State& state = component.current();
        for (int var = 0; var < m_task->get_num_variables(); var++) {
            conjs.push_back(strips::get_fact_id(var, state[var]));
        }
        assert(lookup.contains_subset_of(conjs));
        assert(formula.contains_subset_of(conjs));
        conjs.clear();
        component.next();
        // component_size++;
    }

    // std::cout << "Inserted " << num << " (" << component_size << ") conjunctions" << std::endl;

#endif

    if (c_recompute_reachability) {
        m_trap->update_reachability_insert_conjunctions();
    }

    return true;
}

}
}

static std::shared_ptr<conflict_driven_learning::ConflictLearner>
_parse(options::OptionParser& parser)
{
    conflict_driven_learning::traps::TrapNeighborsRefinement::add_options_to_parser(parser);
    options::Options opts = parser.parse();
    if (!parser.dry_run()) {
        return std::make_shared<conflict_driven_learning::traps::TrapNeighborsRefinement>(opts);
    }
    return nullptr;
}

static Plugin<conflict_driven_learning::ConflictLearner> _plugin("trapsrn", _parse);

