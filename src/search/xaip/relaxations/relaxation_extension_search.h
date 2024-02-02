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

    std::shared_ptr<Evaluator> eval;

    std::shared_ptr<PruningMethod> pruning_method;

    MSGSCollection current_msgs;

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
    void expand(const State &state);
    bool next_relaxed_task();
    bool decide_to_put_into_openlist(const SearchNode &node, const State &state, OperatorID op);

public:
    explicit RelaxationExtensionSearch(const options::Options &opts);
    virtual ~RelaxationExtensionSearch() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}

#endif
