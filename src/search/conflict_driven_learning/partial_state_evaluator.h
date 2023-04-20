#ifndef PARTIAL_STATE_EVALUATOR_H
#define PARTIAL_STATE_EVALUATOR_H

#include <vector>

namespace conflict_driven_learning {

using PartialState = std::vector<std::pair<int, int> >;

class PartialStateEvaluator {
public:
    virtual int evaluate_partial_state(const PartialState& partial_state) = 0;
};

}

#endif
