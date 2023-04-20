#include "quantitative_state_minimization_nogoods.h"

#include "strips_compilation.h"
#include "../abstract_task.h"

#include <algorithm>
#include <iostream>
#include <cstdio>

namespace conflict_driven_learning
{
namespace hc_heuristic
{

int QuantitativeStateMinimizationNoGoods::evaluate_quantitative(
    const std::vector<unsigned> &conjunction_ids)
{
    int res = 0;
    m_formula.forall_subsets(conjunction_ids, [&res, this](unsigned id) {
        if (m_clause_value[id] == HCHeuristic::DEAD_END) {
            res = HCHeuristic::DEAD_END;
            return true;
        }
        res = std::max(res, m_clause_value[id]);
        return false;
    });
    return res;
}

void QuantitativeStateMinimizationNoGoods::initialize()
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
QuantitativeStateMinimizationNoGoods::setup_var_orders()
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

void QuantitativeStateMinimizationNoGoods::notify_on_new_conjunction(unsigned cid)
{
    const std::vector<unsigned>& conj = m_hc->get_conjunction(cid);
    if (std::includes(m_full_goal_facts.begin(), m_full_goal_facts.end(),
                      conj.begin(), conj.end())) {
        m_full_goal_conjunction_ids.push_back(cid);
    }
}

void QuantitativeStateMinimizationNoGoods::refine_quantitative(
    const GlobalState &state,
    int bound)
{
    bool term = m_hc->set_early_termination_and_nogoods(false);

    std::vector<unsigned> facts;
    std::vector<unsigned> previous_facts(facts);
    std::vector<unsigned> conjs;

    for (unsigned i = 0;;) {
        assert(i < m_var_orders.size());
        assert(m_var_orders[i].size() == (unsigned) m_task->get_num_variables());

        const std::vector<int>& order = m_var_orders[i];
        for (int var = 0; var < m_task->get_num_variables(); var++) {
            facts.push_back(strips::get_fact_id(var, state[var]));
        }
        int finalh = 0;
        std::map<unsigned, int> goal_cost;
        for (unsigned j = 0; j < order.size(); j++) {
            int var = order[j];
            previous_facts = facts;
            for (int val = 0; val < m_task->get_variable_domain_size(var); val++) {
                if (val != state[var]) {
                    facts.push_back(strips::get_fact_id(var, val));
                }
            }
            std::sort(facts.begin(), facts.end());
            conjs.clear();
            m_hc->cleanup_previous_computation();
            m_hc->get_satisfied_conjunctions(facts, conjs);
            int res = m_hc->compute_heuristic(conjs);
            if (res != HCHeuristic::DEAD_END && (bound < 0 || res < bound)) {
                m_clause.push_back(strips::get_fact_id(var, state[var]));
                facts.swap(previous_facts);
            } else {
                finalh = res;
                for (const auto& conj_id : m_full_goal_conjunction_ids) {
                    goal_cost[conj_id] = m_hc->get_conjunction_data(conj_id).cost;
                }
            }
        }
        std::sort(m_clause.begin(), m_clause.end());

        conjs.clear();
        m_formula.insert(m_clause);
        m_clause_to_goal_cost.emplace_back(std::move(goal_cost));
        m_clause_value.push_back(finalh);
        m_clause.clear();
        facts.clear();
        if (++i == m_var_orders.size()) {
            break;
        }
    }

    m_hc->set_early_termination_and_nogoods(term);
}

void QuantitativeStateMinimizationNoGoods::synchronize_goal(std::shared_ptr<AbstractTask> task)
{
    m_task = task;

    static std::vector<unsigned> goal_conjunctions;

    for (int i = m_clause_value.size() - 1; i >= 0; i--) {
        m_clause_value[i] = 0;
    }
    m_hc->get_satisfied_conjunctions(strips::get_task().get_goal(),
                                     goal_conjunctions);
    for (const unsigned& conj_id : goal_conjunctions) {
        for (int i = m_clause_value.size() - 1; i >= 0; i--) {
            auto it = m_clause_to_goal_cost[i].find(conj_id);
            if (it != m_clause_to_goal_cost[i].end()) {
                if (it->second == ConjunctionData::UNACHIEVED) {
                    m_clause_value[i] = HCHeuristic::DEAD_END;
                    break;
                } else {
                    m_clause_value[i] = std::max(m_clause_value[i], it->second);
                }
            }
        }
    }
    goal_conjunctions.clear();

    setup_var_orders();
}

void QuantitativeStateMinimizationNoGoods::print_statistics() const
{
    printf("hC-nogood (state minimization) size: %zu\n", m_formula.size());
    std::cout << "hC-nogood (state minimization) evaluation time: "
           << get_evaluation_timer() << std::endl;
    std::cout << "hC-nogood (state minimization) refinement time: "
           << get_refinement_timer() << std::endl;
}

}
}
