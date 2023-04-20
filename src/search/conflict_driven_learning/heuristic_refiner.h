#ifndef HEURISTIC_REFINER_H
#define HEURISTIC_REFINER_H

#include "state_component.h"
#include "../evaluator.h"
#include "../utils/timer.h"

#include <memory>

namespace conflict_driven_learning {

class HeuristicRefiner
{
private:
    bool m_initialized;
    utils::Timer m_refinement_timer;
protected:
    virtual bool refine_heuristic(int bound,
                                  StateComponent& component,
                                  SuccessorComponent& neighbors) = 0;
    virtual void initialize();
    void start_refinement();
    void stop_refinement();
public:
    HeuristicRefiner();
    virtual ~HeuristicRefiner() = default;

    virtual std::shared_ptr<Evaluator> get_underlying_heuristic() = 0;
    virtual bool requires_neighbors() const;
    virtual void print_statistics() const;

    bool notify(int bound,
                StateComponent& component,
                SuccessorComponent& successors);

    bool notify(int bound,
                StateComponent&& component,
                SuccessorComponent&& successors);

    const utils::Timer &get_refinement_timer() const;
};

}

#endif
