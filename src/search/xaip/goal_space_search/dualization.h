#ifndef SEARCH_ENGINES_DUALIZATIONT_SEARCH_H
#define SEARCH_ENGINES_DUALIZATIONT_SEARCH_H

#include "../option_parser_util.h"
#include "../search_engine.h"

#include "../options/registries.h"
#include "../options/predefinitions.h"

#include "goal_subset_space.h"
#include "../goal_subsets/goal_subsets.h"
#include "../goal_subsets/goal_subset.h"

namespace options {
class Options;
}

class Heuristic;

namespace dualization_search {
class DualizationSearch : public SearchEngine {
    const ParseTree engine_config;
    /*
      We need to copy the registry and predefinitions here since they live
      longer than the objects referenced in the constructor.
    */
    options::Registry registry;
    options::Predefinitions predefinitions;
    
    bool all_soft_goals;
    bool weakening;

    int num_planner_calls = 0;

    std::vector<Heuristic *> heuristic;

    std::vector<FactPair> soft_goal_list;
    std::vector<FactPair> hard_goal_list;

    std::vector<std::string> soft_goal_fact_names;

    goalsubset::GoalSubset satisfiable_set;
    goalsubset::GoalSubset maximal_satisfiable_set;

    GoalSubsets current_MUGS;
    GoalSubsets comp_current_MSGS;
    GoalSubsets candidates_MUGS;

    GoalSubsets planner_called_solvable;
    GoalSubsets planner_called_unsolvable;

    bool call_search_engine(goalsubset::GoalSubset set);

    goalsubset::GoalSubset grow(goalsubset::GoalSubset set);
    std::vector<FactPair>  get_goals(goalsubset::GoalSubset set) const;

    virtual SearchStatus step() override;

public:
    DualizationSearch(const options::Options &opts, options::Registry &registry,
                   const options::Predefinitions &predefinitions);

    virtual void save_plan_if_necessary() override;
    virtual void print_statistics() const override;
};
}

#endif
