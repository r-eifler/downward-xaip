#ifndef CEGAR_ADDITIVE_CARTESIAN_HEURISTIC_H
#define CEGAR_ADDITIVE_CARTESIAN_HEURISTIC_H

#include "../heuristic.h"

#include <vector>

namespace cegar {
class CartesianHeuristicFunction;

/*
  Store CartesianHeuristicFunctions and compute overall heuristic by
  summing all of their values.
*/
class AdditiveCartesianHeuristic : public Heuristic {
    
protected:
    const std::vector<CartesianHeuristicFunction> heuristic_functions;
    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    explicit AdditiveCartesianHeuristic(const options::Options &opts);
    std::vector<int> get_heuristic_values(const State &state, std::vector<FactPair> facts) override;
};
}

#endif
