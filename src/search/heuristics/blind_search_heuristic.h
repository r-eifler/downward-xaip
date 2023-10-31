#ifndef HEURISTICS_BLIND_SEARCH_HEURISTIC_H
#define HEURISTICS_BLIND_SEARCH_HEURISTIC_H

#include <vector>

#include "../heuristic.h"

namespace blind_search_heuristic {
class BlindSearchHeuristic : public Heuristic {
    int min_operator_cost;
protected:
    virtual int compute_heuristic(const State &ancestor_state) override;
    std::vector<int> get_heuristic_values(const State &state, std::vector<FactPair> facts) override;
public:
    BlindSearchHeuristic(const options::Options &opts);
    ~BlindSearchHeuristic();
};
}

#endif
