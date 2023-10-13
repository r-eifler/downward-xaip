#ifndef PRUNING_METHOD_H
#define PRUNING_METHOD_H

#include "operator_id.h"
#include "state_id.h"

#include "../utils/logging.h"
#include "../utils/timer.h"

#include "xaip/explicit_mugs_search/msgs_collection.h"

#include <memory>
#include <vector>

class AbstractTask;
class State;

namespace limited_pruning {
class LimitedPruning;
}

namespace options {
class OptionParser;
class Options;
}

class PruningMethod {
    utils::Timer timer_ops;
    utils::Timer timer_states;
    friend class limited_pruning::LimitedPruning;

    virtual void prune(
        const State &state, std::vector<OperatorID> &op_ids) = 0;
    virtual bool prune(const State &state, int remaining_cost);

protected:
    mutable utils::LogProxy log;
    std::shared_ptr<AbstractTask> task;
    long num_successors_before_pruning;
    long num_successors_after_pruning;
    long num_tested_states;
    long num_pruned_states;
public:
    explicit PruningMethod(const options::Options &opts);
    virtual ~PruningMethod() = default;
    virtual void initialize(const std::shared_ptr<AbstractTask> &task);
    void prune_operators(const State &state, std::vector<OperatorID> &op_ids);
    bool prune_state(const State &state, int remaining_cost);
    virtual StateID get_cardinally_best_state();
    virtual int get_max_solved_soft_goals() {return 0;}
    virtual void print_statistics() const;

    virtual void set_abstract_task(std::shared_ptr<AbstractTask> task);

    virtual void notify_initial_state(const State & /*initial_state*/) {}

    virtual void notify_state_transition(
        const State & /*parent_state*/,
        OperatorID /*op_id*/,
        const State & /*state*/) {}

    virtual MSGSCollection get_msgs() const;
    virtual void init_msgs(MSGSCollection collection);
};

extern void add_pruning_options_to_parser(options::OptionParser &parser);

#endif
