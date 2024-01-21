#ifndef DISTANCE_FUNCTION_H
#define DISTANCE_FUNCTION_H

#include "../../operator_id.h"
#include "../../state_id.h"

#include "../utils/logging.h"

#include <memory>
#include <vector>

class AbstractTask;
class State;

namespace options {
class OptionParser;
class Options;
}

class DistanceFunction {

protected:
    mutable utils::LogProxy log;
    std::shared_ptr<AbstractTask> task;
public:
    explicit DistanceFunction(const options::Options &opts);
    virtual ~DistanceFunction() = default;
    virtual void initialize(const std::shared_ptr<AbstractTask> &task);
    virtual std::vector<float> get_distances(const State &state, std::vector<OperatorID> &op_ids);
    virtual void print_statistics() const;

    virtual void notify_initial_state(const State & /*initial_state*/) {}

    virtual void notify_state_transition(
        const State & /*parent_state*/,
        OperatorID /*op_id*/,
        const State & /*state*/) {}

};

#endif
