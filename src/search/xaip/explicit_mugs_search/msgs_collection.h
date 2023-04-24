#ifndef MSGS_COLLECTION_H
#define MSGS_COLLECTION_H

#include "../goal_subsets/goal_subsets.h"
#include "../../task_proxy.h"
#include "../../tasks/root_task.h"
#include <iostream>
#include <string>
#include <unordered_set>


class MSGSCollection : GoalSubsets{

private: 

    std::shared_ptr<AbstractTask> task;

    std::vector<FactPair> soft_goal_list;
    std::vector<FactPair> hard_goal_list;
    std::vector<FactPair> all_goal_list;

    std::vector<std::string> soft_goal_fact_names;

protected:

    goalsubset::GoalSubset get_satisfied_soft_goals(const State &state);
    goalsubset::GoalSubset get_satisfied_hard_goals(const State &state);
    goalsubset::GoalSubset get_satisfied_all_goals(const State &state);

    goalsubset::GoalSubset get_reachable_soft_goals(goalsubset::GoalSubset reachable_goals);
    goalsubset::GoalSubset get_reachable_hard_goals(goalsubset::GoalSubset reachable_goals);

    void add_and_mimize(goalsubset::GoalSubset subset);
    bool contains_superset(goalsubset::GoalSubset subset);
    bool contains_strict_superset(goalsubset::GoalSubset subset);

public:
    explicit MSGSCollection();
    void initialize(std::shared_ptr<AbstractTask> task);

    std::vector<FactPair> get_goal_facts();

    bool prune(const State &state, std::vector<int> costs, int remaining_cost);
    bool track(const State &state);

    void print() const;
};


#endif 
