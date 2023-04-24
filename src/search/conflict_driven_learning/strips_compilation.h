#ifndef STRIPS_COMPILATION_H
#define STRIPS_COMPILATION_H

#include "../abstract_task.h"

#include <vector>
#include <utility>
#include <string>

class State;

namespace conflict_driven_learning
{
namespace strips
{

struct Action {
    int cost;
    std::vector<unsigned> pre;
    std::vector<unsigned> add;
    std::vector<unsigned> del;
};

struct Task {
    std::vector<Action> m_actions;
    std::vector<std::vector<unsigned> > m_actions_with_add;
    std::vector<std::vector<unsigned> > m_actions_with_del;
    std::vector<std::vector<unsigned> > m_actions_with_mutex_regression;
    std::vector<unsigned> m_goal;
    std::vector<std::vector<bool> > m_mutexes;
    const std::vector<unsigned> &get_goal() const
    {
        return m_goal;
    }
    const Action &get_action(unsigned op) const
    {
        assert(op < m_actions.size());
        return m_actions[op];
    }
    const std::vector<unsigned> &get_actions_with_add(unsigned p) const
    {
        assert(p < m_actions_with_add.size());
        return m_actions_with_add[p];
    }
    const std::vector<unsigned> &get_actions_with_del(unsigned p) const
    {
        assert(p < m_actions_with_del.size());
        return m_actions_with_del[p];
    }
    const std::vector<unsigned> &get_actions_with_mutex_regression(unsigned p) const
    {
        assert(p < m_actions_with_mutex_regression.size());
        return m_actions_with_mutex_regression[p];
    }
    std::size_t num_actions() const
    {
        return m_actions.size();
    }
    bool contains_mutex(const std::vector<unsigned> &subgoal) const;
    bool get_mutex(const std::vector<unsigned> &subgoal,
                   std::vector<unsigned> &conflict) const;
    bool are_mutex(const std::vector<unsigned> &x,
                   const std::vector<unsigned> &y) const;
    bool are_mutex(const unsigned &x, const unsigned &y) const;
    bool are_mutex(const unsigned &x, const std::vector<unsigned> &y) const;
    void compute_achievers(std::vector<int> &achievers,
                           const std::vector<unsigned> &conj) const;
    std::size_t generate_singleton_effect_actions(
        std::vector<std::vector<unsigned> > &preconditions,
        std::vector<std::vector<unsigned> > &achievers,
        std::vector<std::vector<unsigned> > &actions_with_pre,
        std::vector<unsigned> &original_action) const;
    static bool are_mutex(const std::vector<std::vector<bool> > &mutexes,
                          const std::vector<unsigned> &x,
                          const std::vector<unsigned> &y);
};

void initialize(const AbstractTask& task);
void update_goal_set(const AbstractTask& task);
const Task &get_task();
unsigned get_fact_id(int var, int val);
template<typename t_var, typename t_val>
unsigned get_fact_id(const std::pair<t_var, t_val> &assignment)
{
    return get_fact_id(assignment.first, assignment.second);
}
std::pair<int, int> get_variable_assignment(const unsigned& fact_id);
std::size_t num_facts();
void get_fact_ids(std::vector<unsigned>& fact_ids, const State& state);
void get_fact_ids(std::vector<unsigned>& fact_ids, const std::vector<std::pair<int, int> >& state);

}
}

#endif
