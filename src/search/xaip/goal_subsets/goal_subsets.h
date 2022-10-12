#ifndef GOAL_SUBSETS_H
#define GOAL_SUBSETS_H

#include "../../task_proxy.h"
#include "goal_subset.h"
#include <iostream>
#include <string>
#include <unordered_set>

typedef std::unordered_set<goalsubset::GoalSubset, goalsubset::GoalSubsetHashFunction, goalsubset::GoalSubsetEqualFunction> GoalSubsetHashSet;


class GoalSubsets {

private: 

    GoalSubsetHashSet subsets;

    std::vector<std::vector<std::string>> generate_string(std::vector<std::string> goal_facts_names);
    void print_subsets(std::vector<std::vector<std::string>> facts_names);
    std::string to_json(std::vector<std::vector<std::string>> facts_names);

public:
    explicit GoalSubsets();
    explicit GoalSubsets(GoalSubsetHashSet subsets);

    size_t size() const {
        return subsets.size();
    }

    void add(goalsubset::GoalSubset subset);
    void add(GoalSubsetHashSet subsets);

    void print(std::vector<std::string> goal_facts_names);
    void print_subsets();
    void to_file(std::vector<std::string> goal_facts_names, std::string file_name = "mugs.json");

    void minimize_non_minimal_subsets();
    GoalSubsets cross_product(GoalSubsets sets) const;
    GoalSubsets complement() const;
    GoalSubsets minimal_hitting_sets();
};


#endif 
