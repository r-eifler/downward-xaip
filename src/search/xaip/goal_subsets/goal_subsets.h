#ifndef GOAL_SUBSETS_H
#define GOAL_SUBSETS_H

#include "../../task_proxy.h"
#include "goal_subset.h"
#include <iostream>
#include <string>
#include <unordered_set>

typedef std::unordered_set<goalsubset::GoalSubset, goalsubset::GoalSubsetHashFunction, goalsubset::GoalSubsetEqualFunction> GoalSubsetHashSet;


class GoalSubsets {

protected: 

    GoalSubsetHashSet subsets;

    void print_subsets(std::vector<std::vector<std::string>> facts_names) const;
    std::string to_json(std::vector<std::vector<std::string>> facts_names);

public:
    explicit GoalSubsets();
    explicit GoalSubsets(GoalSubsetHashSet subsets);

    using iterator= typename GoalSubsetHashSet::iterator;

    iterator begin(){ return subsets.begin(); };
    iterator end(){ return subsets.end(); };

    size_t size() const {
        return subsets.size();
    }

    void add(goalsubset::GoalSubset subset);
    bool contains(goalsubset::GoalSubset subset);
    void add(GoalSubsetHashSet subsets);
    void add(GoalSubsets subsets);

    void print(std::vector<std::string> goal_facts_names) const;
    void print_subsets() const;
    void to_file(std::vector<std::string> goal_facts_names, std::string file_name = "mugs.json");
    std::vector<std::vector<std::string>> generate_string(std::vector<std::string> goal_facts_names) const;

    void minimize_non_minimal_subsets();
    void minimize_non_maximal_subsets();
    GoalSubsets cross_product(GoalSubsets sets) const;
    GoalSubsets complement() const;
    GoalSubsets minus(GoalSubsets sets) const;
    GoalSubsets minimal_hitting_sets();
};


#endif 
