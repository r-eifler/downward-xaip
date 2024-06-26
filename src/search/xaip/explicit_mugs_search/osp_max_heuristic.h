#ifndef HEURISTICS_OSP_MAX_HEURISTIC_H
#define HEURISTICS_OSP_MAX_HEURISTIC_H

#include "../../heuristics/max_heuristic.h"

class OSPMaxHeuristic : public max_heuristic::HSPMaxHeuristic {

protected:
    virtual int compute_heuristic(const State &ancestor_state) override;
public:
    explicit OSPMaxHeuristic(const options::Options &opts);
};


#endif
