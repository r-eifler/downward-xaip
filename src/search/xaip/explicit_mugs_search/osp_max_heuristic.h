#ifndef HEURISTICS_OSP_MAX_HEURISTIC_H
#define HEURISTICS_OSP_MAX_HEURISTIC_H

#include "../../heuristics/max_heuristic.h"

#include "../algorithms/priority_queues.h"

#include <cassert>
#include <boost/dynamic_bitset.hpp>


class OSPMaxHeuristic : public max_heuristic::HSPMaxHeuristic {

protected:
    virtual int compute_heuristic(const State &ancestor_state) override;
public:
    explicit OSPMaxHeuristic(const options::Options &opts);
};


#endif
