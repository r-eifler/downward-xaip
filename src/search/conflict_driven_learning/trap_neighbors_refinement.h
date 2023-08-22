#ifndef TRAP_NEIGHBORS_REFINEMENT_H
#define TRAP_NEIGHBORS_REFINEMENT_H

#include "conflict_learner.h"

#include <vector>

class AbstractTask;

namespace options {
class Options;
class OptionParser;
}

namespace conflict_driven_learning {
namespace traps {

class TrapUnsatHeuristic;

class TrapNeighborsRefinement : public ConflictLearner {
public:
    TrapNeighborsRefinement(const options::Options& opts);
    
    virtual Evaluator* get_underlying_heuristic() override;
    virtual bool requires_recognized_neighbors() const override;

    static void add_options_to_parser(options::OptionParser& parser);
protected:
    virtual bool learn_from_dead_end_component(StateComponent &, StateComponent &) override;

    const bool c_recompute_reachability;

    std::shared_ptr<TrapUnsatHeuristic> m_trap;
    const AbstractTask* m_task;
    std::vector<bool> m_fact_mutex_with_goal;
    std::vector<int> m_variable_order;
};

}
}

#endif
