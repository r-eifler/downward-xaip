#ifndef SEARCH_ENGINES_RELAXATION_ITERATED_SEARCH_H
#define SEARCH_ENGINES_RELAXATION_ITERATED_SEARCH_H

#include "../option_parser_util.h"
#include "../search_engine.h"
#include "task_relaxation_tracker.h"
#include "relaxed_task.h"
#include "../options/registries.h"
#include "../options/predefinitions.h"

namespace options {
class Options;
}

class Heuristic;

namespace dwq_iterated_search {
class RelaxationIteratedSearch : public SearchEngine {
    const std::vector<options::ParseTree> engine_configs;

    options::Registry registry;
    options::Predefinitions predefinitions;

    bool propagate_msgs;

    TaskRelaxationTracker* taskRelaxationTracker;
    RelaxedTask* relaxedTask = NULL;
    std::vector<Heuristic *> heuristic;

    std::shared_ptr<SearchEngine> get_search_engine();
    std::shared_ptr<SearchEngine> create_phase();
    SearchStatus step_return_value();

    virtual SearchStatus step() override;

public:
    explicit RelaxationIteratedSearch(const options::Options &opts, options::Registry &registry,
                   const options::Predefinitions &predefinitions);

    virtual void save_plan_if_necessary() override;
    virtual void print_statistics() const override;
};
}

#endif
