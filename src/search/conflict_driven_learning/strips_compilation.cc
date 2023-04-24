
#include "strips_compilation.h"
#include "../task_proxy.h"
#include "../task_utils/task_properties.h"
#include "../global_state.h"

#include <utility>
#include <algorithm>
#include <iostream>
#include <cstdio>

namespace conflict_driven_learning
{
namespace strips
{

static bool _initialized = false;
static size_t _num_facts = -1;
static Task strips_task;
static std::vector<unsigned> variable_offset;
static const AbstractTask* abstract_task_ref = NULL;

void get_fact_ids(std::vector<unsigned>& fact_ids, const State& state)
{
    for (unsigned var = 0; var < variable_offset.size(); var++) {
        fact_ids.push_back(variable_offset[var] + state[var]);
    }
}

void get_fact_ids(std::vector<unsigned>& fact_ids, const std::vector<std::pair<int, int> >& state)
{
    for (unsigned i = 0; i < state.size(); i++) {
        fact_ids.push_back(variable_offset[state[i].first] + state[i].second);
    }
}

std::pair<int, int> get_variable_assignment(const unsigned& fact_id)
{
    std::pair<int, int> res(0, fact_id);
    while (res.first < (int) variable_offset.size()
            && (res.second - (int) variable_offset[res.first]) >= 0) {
        res.first++;
    }
    res.first--;
    res.second = res.second - variable_offset[res.first];
    return res;
}

unsigned get_fact_id(int var, int val)
{
    assert(_initialized);
    return variable_offset[var] + val;
}

size_t num_facts()
{
    assert(_initialized);
    return _num_facts;
}

const Task &get_task()
{
    assert(_initialized);
    return strips_task;
}

void initialize(const AbstractTask& task)
{
#if 0
    if (&task == abstract_task_ref) {
        return;
    }

    if (_initialized) {
        strips_task.m_actions.clear();
        strips_task.m_actions_with_add.clear();
        strips_task.m_actions_with_del.clear();
        strips_task.m_actions_with_mutex_regression.clear();
        strips_task.m_goal.clear();
        strips_task.m_mutexes.clear();
        variable_offset.clear();
        _num_facts = -1;
    }
#else
    if (_initialized) {
        if (abstract_task_ref != &task) {
            abstract_task_ref = &task;
            update_goal_set(task);
        }
        return;
    }
#endif

    _initialized = true;
    abstract_task_ref = &task;

    /* std::cout << "Converting FDR to STRIPS ..." << std::endl; */
    /* utils::Timer tconversion; */

    TaskProxy task_proxy(task);
    task_properties::verify_no_axioms(task_proxy);
    task_properties::verify_no_conditional_effects(task_proxy);

    _num_facts = task.get_variable_domain_size(0);
    variable_offset.resize(task.get_num_variables(), 0);
    for (unsigned var = 1; var < variable_offset.size(); var++) {
        variable_offset[var] = _num_facts;
        _num_facts += task.get_variable_domain_size(var);
    }

    std::vector<std::vector<unsigned> > mutex_with(_num_facts);
    strips_task.m_mutexes.resize(_num_facts, std::vector<bool>(_num_facts, false));
    for (int var1 = 0; var1 < task.get_num_variables(); var1++) {
        for (int val1 = 0; val1 < task.get_variable_domain_size(var1); val1++) {
            unsigned p = variable_offset[var1] + val1;
            FactPair p1(var1, val1);
            for (int var2 = var1; var2 < task.get_num_variables(); var2++) {
                for (int val2 = 0; val2 < task.get_variable_domain_size(var2); val2++) {
                    FactPair p2(var2, val2);
                    if (task.are_facts_mutex(p1, p2)) {
                        unsigned q = variable_offset[var2] + val2;
                        strips_task.m_mutexes[p][q] = true;
                        strips_task.m_mutexes[q][p] = true;
                        mutex_with[p].push_back(q);
                        mutex_with[q].push_back(p);
                    }
                }
            }
        }
    }

    strips_task.m_actions.resize(task.get_num_operators());
    strips_task.m_actions_with_add.resize(_num_facts);
    strips_task.m_actions_with_del.resize(_num_facts);
    strips_task.m_actions_with_mutex_regression.resize(_num_facts);

    std::vector<int> var_in_pre(task.get_num_variables(), -1);
    std::vector<bool> regression_is_mutex_with(_num_facts);
    for (int op = 0; op < task.get_num_operators(); op++) {
        std::fill(var_in_pre.begin(), var_in_pre.end(), -1);
        std::fill(regression_is_mutex_with.begin(),
                  regression_is_mutex_with.end(),
                  false);

        Action &action = strips_task.m_actions[op];
        action.cost = task.get_operator_cost(op, false);

        for (int i = 0; i < task.get_num_operator_preconditions(op, false); i++) {
            FactPair p = task.get_operator_precondition(op, i, false);
            var_in_pre[p.var] = p.value;
            action.pre.push_back(variable_offset[p.var] + p.value);
            for (const unsigned &q : mutex_with[action.pre.back()]) {
                regression_is_mutex_with[q] = true;
            }
        }

        for (int i = 0; i < task.get_num_operator_effects(op, false); i++) {
            bool adds = true;
            FactPair e = task.get_operator_effect(op, i, false);
            if (var_in_pre[e.var] != -1) {
                if (var_in_pre[e.var] == e.value) {
                    adds = false;
                } else {
                    unsigned p = variable_offset[e.var] + var_in_pre[e.var];
                    action.del.push_back(p);
                    strips_task.m_actions_with_del[p].push_back(op);
                }
            } else {
                for (int val = 0; val < task.get_variable_domain_size(e.var); val++) {
                    if (val != e.value) {
                        action.del.push_back(variable_offset[e.var] + val);
                        strips_task.m_actions_with_del[variable_offset[e.var] + val].push_back(op);
                    }
                }
            }
            if (adds) {
                action.add.push_back(variable_offset[e.var] + e.value);
                strips_task.m_actions_with_add[variable_offset[e.var] + e.value].push_back(op);
            }
        }

        for (const unsigned &p : action.add) {
            regression_is_mutex_with[p] = false;
        }

        for (unsigned p = 0; p < regression_is_mutex_with.size(); p++) {
            if (regression_is_mutex_with[p]) {
                strips_task.m_actions_with_mutex_regression[p].push_back(op);
            }
        }

        std::sort(action.pre.begin(), action.pre.end());
        action.pre.erase(std::unique(action.pre.begin(), action.pre.end()), action.pre.end());
        std::sort(action.add.begin(), action.add.end());
        action.add.erase(std::unique(action.add.begin(), action.add.end()), action.add.end());
        std::sort(action.del.begin(), action.del.end());
        action.del.erase(std::unique(action.del.begin(), action.del.end()), action.del.end());
    }

    update_goal_set(task);

    /* printf("Convertion completed in %.4fs.\n", tconversion()); */
    /* printf("STRIPS task consists of %zu facts and %zu actions.\n", */
    /*        _num_facts, */
    /*        strips_task.num_actions()); */
}

void update_goal_set(const AbstractTask& task)
{
    strips_task.m_goal.clear();
    strips_task.m_goal.reserve(task.get_num_goals());
    for (int i = 0; i < task.get_num_goals(); i++) {
        auto g = task.get_goal_fact(i);
        strips_task.m_goal.push_back(variable_offset[g.var] + g.value);
    }
    std::sort(strips_task.m_goal.begin(), strips_task.m_goal.end());
}

bool Task::contains_mutex(const std::vector<unsigned> &subgoal) const
{
    for (unsigned i = 0; i < subgoal.size() - 1; i++) {
        assert(subgoal[i] < num_facts());
        const std::vector<bool> &mutexes = m_mutexes[subgoal[i]];
        for (unsigned j = i + 1; j < subgoal.size(); j++) {
            assert(subgoal[j] < num_facts());
            if (mutexes[subgoal[j]]) {
                return true;
            }
        }
    }
    return false;
}

bool Task::get_mutex(const std::vector<unsigned> &subgoal,
                     std::vector<unsigned> &conflict) const
{
    for (unsigned i = 0; i < subgoal.size() - 1; i++) {
        const std::vector<bool> &mutexes = m_mutexes[subgoal[i]];
        for (unsigned j = i + 1; j < subgoal.size(); j++) {
            if (mutexes[subgoal[j]]) {
                conflict.push_back(subgoal[i]);
                conflict.push_back(subgoal[j]);
                return true;
            }
        }
    }
    return false;
}

bool Task::are_mutex(const std::vector<std::vector<bool> > &mutexes,
                     const std::vector<unsigned> &x,
                     const std::vector<unsigned> &y)
{
    for (unsigned i = 0; i < x.size(); i++) {
        for (unsigned j = 0; j < y.size(); j++) {
            if (mutexes[x[i]][y[j]]) {
                return true;
            }
        }
    }
    return false;
}

bool Task::are_mutex(const std::vector<unsigned> &x,
                     const std::vector<unsigned> &y) const
{
    return are_mutex(m_mutexes, x, y);
}

bool Task::are_mutex(const unsigned &x, const unsigned &y) const
{
    return m_mutexes[x][y];
}

bool Task::are_mutex(const unsigned &x, const std::vector<unsigned> &y) const
{
    for (const unsigned &z : y) {
        if (m_mutexes[x][z]) {
            return true;
        }
    }
    return false;
}

void Task::compute_achievers(std::vector<int> &achievers,
                             const std::vector<unsigned> &conj) const
{
    assert(achievers.size() == num_actions());
    for (const unsigned &p : conj) {
        for (const unsigned &op : m_actions_with_mutex_regression[p]) {
            achievers[op] = -1;
        }
        for (const unsigned &op : get_actions_with_del(p)) {
            achievers[op] = -1;
        }
        for (const unsigned &op : get_actions_with_add(p)) {
            if (achievers[op] == 0) {
                achievers[op] = 1;
            }
        }
    }
}

size_t Task::generate_singleton_effect_actions(
    std::vector<std::vector<unsigned> > &preconditions,
    std::vector<std::vector<unsigned> > &achievers,
    std::vector<std::vector<unsigned> > &actions_with_pre,
    std::vector<unsigned> &original_action) const
{
    unsigned num_counters = preconditions.size();
    for (unsigned op = 0; op < num_actions(); op++) {
        const Action &action = m_actions[op];
        for (const unsigned &p : action.add) {
            unsigned counterid = num_counters++;
            preconditions.push_back(action.pre);
            achievers[p].push_back(counterid);
            original_action.push_back(op);
            for (const unsigned &q : action.pre) {
                actions_with_pre[q].push_back(counterid);
            }
        }
    }
    return num_counters;
}

}
}
