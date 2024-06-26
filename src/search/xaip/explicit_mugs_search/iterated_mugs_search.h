#ifndef SEARCH_ENGINES_ITERATED_MUGS_SEARCH_H
#define SEARCH_ENGINES_ITERATED_MUGS_SEARCH_H

#include "../option_parser_util.h"
#include "../search_engine.h"
#include "../../pruning_method.h"

#include "../options/registries.h"
#include "../options/predefinitions.h"

namespace options {
class Options;
}

namespace iterated_mugs_search {
class IteratedMUGSSearch : public SearchEngine {
    const std::vector<options::ParseTree> engine_configs;
    /*
      We need to copy the registry and predefinitions here since they live
      longer than the objects referenced in the constructor.
    */
    options::Registry registry;
    options::Predefinitions predefinitions;
    bool pass_bound;

    int phase;
    bool iterated_found_solution;

    std::shared_ptr<PruningMethod> pruning_method;

    virtual void initialize() override;
    std::shared_ptr<SearchEngine> get_search_engine(int engine_configs_index);
    std::shared_ptr<SearchEngine> create_current_phase();
    SearchStatus step_return_value();

    virtual SearchStatus step() override;

public:
    IteratedMUGSSearch(const options::Options &opts, options::Registry &registry,
                   const options::Predefinitions &predefinitions);

    virtual void save_plan_if_necessary() override;
    virtual void print_statistics() const override;
};
}

#endif
