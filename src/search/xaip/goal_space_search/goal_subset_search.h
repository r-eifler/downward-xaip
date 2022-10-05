#ifndef SEARCH_ENGINES_GOAL_SUBSET_SEARCH_H
#define SEARCH_ENGINES_GOAL_SUBSET_SEARCH_H

#include "../option_parser_util.h"
#include "../search_engine.h"

#include "../options/registries.h"
#include "../options/predefinitions.h"

#include "goal_subset_space.h"

namespace options {
class Options;
}

class Heuristic;

namespace goal_subset_search {
class GoalSubsetSearch : public SearchEngine {
    const ParseTree engine_config;
    /*
      We need to copy the registry and predefinitions here since they live
      longer than the objects referenced in the constructor.
    */
    options::Registry registry;
    options::Predefinitions predefinitions;
    
    bool all_soft_goals;
    bool weakening;

    int num_solved_nodes = 0;

    goalsubsetspace::GoalSubsetSpace* meta_search_space;
    std::vector<Heuristic *> heuristic;

    std::shared_ptr<SearchEngine> get_next_search_engine();

    virtual SearchStatus step() override;

public:
    GoalSubsetSearch(const options::Options &opts, options::Registry &registry,
                   const options::Predefinitions &predefinitions);

    virtual void save_plan_if_necessary() override;
    virtual void print_statistics() const override;
};
}

#endif
