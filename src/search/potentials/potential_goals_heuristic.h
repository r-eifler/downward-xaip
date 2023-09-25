#ifndef POTENTIALS_POTENTIAL_GOALS_HEURISTIC_H
#define POTENTIALS_POTENTIAL_GOALS_HEURISTIC_H

#include "../heuristic.h"

#include <memory>
#include <vector>

namespace potentials {
class PotentialFunction;

/*
  Taskes one potetntial function for each goal fact and resturns the values of all of them
*/
class PotentialGoalsHeuristic : public Heuristic {
    std::vector<std::unique_ptr<PotentialFunction>> functions;

protected:
    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    explicit PotentialGoalsHeuristic(
        const options::Options &opts,
        std::vector<std::unique_ptr<PotentialFunction>> &&functions);
    ~PotentialGoalsHeuristic() = default;
    std::vector<int> get_heuristic_values(const State &ancestor_state, std::vector<FactPair> goals) override;
};
}

#endif
