#ifndef CEGAR_OSP_CARTESIAN_HEURISTIC_H
#define CEGAR_OSP_CARTESIAN_HEURISTIC_H

#include "../../cegar/additive_cartesian_heuristic.h"

#include <vector>

namespace osp_cegar {

/*
  Store CartesianHeuristicFunctions and compute overall heuristic by
  summing all of their values.
*/
class OSPCartesianHeuristic : public cegar::AdditiveCartesianHeuristic {
protected:
    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    explicit OSPCartesianHeuristic(const options::Options &opts);
};
}

#endif
