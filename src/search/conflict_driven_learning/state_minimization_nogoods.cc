#include "state_minimization_nogoods.h"

#include "strips_compilation.h"
#include "../abstract_task.h"

#include <algorithm>
#include <iostream>
#include <cstdio>

namespace conflict_driven_learning
{
namespace hc_heuristic
{

bool StateMinimizationNoGoods::evaluate(
    const std::vector<unsigned> &conjunction_ids)
{
    return m_formula.contains_subset_of(conjunction_ids);
}

void StateMinimizationNoGoods::initialize()
{
    assert(m_full_goal_facts.empty());
    m_formula.set_num_keys(strips::num_facts());
    const auto& goal = strips::get_task().get_goal();
    m_full_goal_facts.insert(m_full_goal_facts.end(), goal.begin(), goal.end());
    m_hc->get_satisfied_conjunctions(m_full_goal_facts, m_full_goal_conjunction_ids);

    m_cur_goal_assignment.resize(m_task->get_num_variables());
    setup_var_orders();
}

void
StateMinimizationNoGoods::setup_var_orders()
{
    std::fill(m_cur_goal_assignment.begin(), m_cur_goal_assignment.end(), -1);
    for (int i = m_task->get_num_goals() - 1; i >= 0; i--) {
        FactPair g = m_task->get_goal_fact(i);
        m_cur_goal_assignment[g.var] = g.value;
    }
    m_var_orders.clear();
    m_var_orders.resize(m_task->get_num_goals());
    for (int i = m_task->get_num_goals() - 1; i >= 0; i--) {
        FactPair g = m_task->get_goal_fact(i);
        std::vector<int>& order = m_var_orders[i];
        order.reserve(m_task->get_num_variables());
        order.push_back(g.var);
        for (int j = m_task->get_num_goals() - 1; j >= 0; j--) {
            if (i != j) {
                order.push_back(m_task->get_goal_fact(j).var);
            }
        }
        for (int var = m_cur_goal_assignment.size() - 1; var >= 0; var--) {
            if (m_cur_goal_assignment[var] == -1) {
                order.push_back(var);
            }
        }
        assert(m_var_orders[i].size() == (unsigned) m_task->get_num_variables());
    }
    m_var_orders.resize(1);
}

void StateMinimizationNoGoods::notify_on_new_conjunction(unsigned cid)
{
    const std::vector<unsigned>& conj = m_hc->get_conjunction(cid);
    if (std::includes(m_full_goal_facts.begin(), m_full_goal_facts.end(),
                      conj.begin(), conj.end())) {
        m_full_goal_conjunction_ids.push_back(cid);
    }
}

void StateMinimizationNoGoods::refine(
    const State &state)
{
    bool term = m_hc->set_early_termination_and_nogoods(false);

    static std::vector<unsigned> new_facts;
    static std::vector<unsigned> reachable;
    for (unsigned i = 0;;) {
        assert(i < m_var_orders.size());
        assert(m_var_orders[i].size() == (unsigned) m_task->get_num_variables());
        const std::vector<int>& order = m_var_orders[i];
        for (unsigned j = 0; j < order.size(); j++) {
            int var = order[j];
            for (int val = 0; val < m_task->get_variable_domain_size(var); val++) {
                if (val != state[var].get_value()) {
                    m_new_facts.push_back(strips::get_fact_id(var, val));
                }
            }
            int res = m_hc->compute_heuristic_incremental(
                    m_new_facts,
                    m_reachable_conjunctions);
            if (res != HCHeuristic::DEAD_END) {
                m_hc->revert_incremental_computation(m_new_facts, m_reachable_conjunctions);
                m_clause.push_back(strips::get_fact_id(var, state[var].get_value()));
            } else {
                new_facts.insert(new_facts.end(), m_new_facts.begin(), m_new_facts.end());
                reachable.insert(reachable.end(), m_reachable_conjunctions.begin(), m_reachable_conjunctions.end());
            }
            m_new_facts.clear();
            m_reachable_conjunctions.clear();
        }
        std::sort(m_clause.begin(), m_clause.end());

        if (m_formula.insert(m_clause).second) {
            unsigned id = m_clauses.size();
            m_clauses.push_back(m_clause);

            for (const auto& conj_id : m_full_goal_conjunction_ids) {
                if (!m_hc->get_conjunction_data(conj_id).achieved()) {
                    m_conjs_to_clauses[conj_id].push_back(id);
                }
            }
        }
        m_clause.clear();
        if (++i == m_var_orders.size()) {
            break;
        } else {
            m_hc->revert_incremental_computation(new_facts, reachable);
            new_facts.clear();
            reachable.clear();
        }
    }
    new_facts.clear();
    reachable.clear();

    m_hc->set_early_termination_and_nogoods(term);
}

void StateMinimizationNoGoods::synchronize_goal(std::shared_ptr<AbstractTask> task)
{
    m_task = task;

    static std::vector<bool> x;
    static std::vector<unsigned> goal_conjunctions;
    x.resize(m_clauses.size());
    std::fill(x.begin(), x.end(), false);

    m_formula.clear();
#if 0
    m_clauses.resize(0);
    m_conjs_to_clauses.clear();
#else
    m_hc->get_satisfied_conjunctions(strips::get_task().get_goal(),
                                     goal_conjunctions);
    for (const unsigned& conj_id : goal_conjunctions) {
        for (const auto& id : m_conjs_to_clauses[conj_id]) {
            if (!x[id]) {
                x[id] = true;
                m_formula.insert(m_clauses[id]);
            }
        }
    }
#endif
    goal_conjunctions.clear();

    setup_var_orders();
}

void StateMinimizationNoGoods::print_statistics() const
{
    printf("hC-nogood (state minimization) size: %zu\n", m_formula.size());
    std::cout << "hC-nogood (state minimization) evaluation time: "
           << get_evaluation_timer() << std::endl;
    std::cout << "hC-nogood (state minimization) refinement time: "
           << get_refinement_timer() << std::endl;
}

}
}
