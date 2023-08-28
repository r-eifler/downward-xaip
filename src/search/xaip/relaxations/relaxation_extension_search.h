#ifndef SEARCH_ENGINES_RELAXATION_EXTENSION_EAGER_SEARCH_H
#define SEARCH_ENGINES_RELAXATION_EXTENSION_SEARCH_H

#include "../../open_list.h"
#include "../../search_engine.h"

#include "relaxed_task.h"
#include "task_relaxation_tracker.h"

#include <memory>
#include <vector>

class Evaluator;
class PruningMethod;

namespace options {
class OptionParser;
class Options;
}

namespace relaxation_extension_search {
class RelaxationExtensionSearch : public SearchEngine {
    const bool reopen_closed_nodes;

    std::unique_ptr<StateOpenList> open_list;
    std::shared_ptr<Evaluator> f_evaluator;

    std::vector<Evaluator *> path_dependent_evaluators;
    std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
    std::shared_ptr<Evaluator> lazy_evaluator;

    std::shared_ptr<PruningMethod> pruning_method;

    TaskRelaxationTracker* taskRelaxationTracker;
    RelaxedTask* relaxedTask;
    std::vector<StateID> pending_initial_states;

    int expanded_states_up_to_prev_task = 0;

    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(EvaluationContext &eval_context);
    void reward_progress();

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;
    bool expand(const State &state);
    bool init_with_frontier_states();
    bool next_relaxed_task();

public:
    explicit RelaxationExtensionSearch(const options::Options &opts);
    virtual ~RelaxationExtensionSearch() = default;

    void set_pruning_method( std::shared_ptr<PruningMethod> pruning_method){
        std::cout << "********** set pruning method ***************" << std::endl;
        this->pruning_method = pruning_method;
    }

    virtual void print_statistics() const override;

    void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}

#endif
