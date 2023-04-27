#ifndef CONFLICT_LEARNER_H
#define CONFLICT_LEARNER_H

#include "state_component.h"
#include "../evaluator.h"

#include "../utils/timer.h"

#include <memory>

namespace conflict_driven_learning
{

class ConflictLearner
{
private:
    bool m_initialized;
    utils::Timer m_refinement_timer;
protected:
    virtual bool learn_from_dead_end_component(StateComponent &, StateComponent &) = 0;
    virtual void initialize();
public:
    ConflictLearner();
    virtual ~ConflictLearner() = default;

    bool notify_dead_end_component(StateComponent &component,
                                   StateComponent &recognized_neighbors);
    bool notify_dead_end_component(StateComponent &&component,
                                   StateComponent &&recognized_neighbors);

    virtual Evaluator* get_underlying_heuristic() = 0;

    virtual bool requires_recognized_neighbors() const;
    virtual void print_statistics() const;

    const utils::Timer &get_refinement_timer() const;

};

}

#endif
