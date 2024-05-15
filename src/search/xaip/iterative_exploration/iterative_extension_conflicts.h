#ifndef SEARCH_ENGINES_RELAXATION_EXTENSION_EAGER_CONFLICT_H
#define SEARCH_ENGINES_RELAXATION_EXTENSION_CONFLICT_SEARCH_H

#include "../../open_list.h"
#include "../../search_engine.h"

#include "radius_tracker.h"
#include "../policy/policy_pruning.h"

#include <memory>
#include <vector>

class Evaluator;
class PruningMethod;

namespace options {
class OptionParser;
class Options;
}

namespace iterative_extension_conflict {
class IterativeExtensionConflict : public SearchEngine {
    const bool reopen_closed_nodes;

    std::unique_ptr<StateOpenList> open_list;
    std::shared_ptr<Evaluator> f_evaluator;

    std::shared_ptr<Evaluator> eval;

    std::shared_ptr<policy_pruning_method::PolicyPruningMethod> pruning_method;

    MSGSCollection current_msgs;

    RadiusTracker radius_tracker;
    float current_radius;
    std::vector<StateID> pending_initial_states;

    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(EvaluationContext &eval_context);
    void reward_progress();

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;
    void expand(const State &state);
    bool next_radius();
    bool decide_to_openlist(const SearchNode &node, const State &state, OperatorID op);

public:
    explicit IterativeExtensionConflict(const options::Options &opts);
    virtual ~IterativeExtensionConflict() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}

#endif
