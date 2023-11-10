#ifndef SEARCH_ENGINES_INITIAL_STATE_HEURISTIC_H
#define SEARCH_ENGINES_INITIAL_STATE_HEURISTIC_H

#include "../search_engine.h"

#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace options {
class Options;
}

namespace initial_state_heuristic_search {

class InitialStateHeurisicSearch : public SearchEngine {

    std::shared_ptr<Evaluator> evaluator;

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    explicit InitialStateHeurisicSearch(const options::Options &opts);
    virtual ~InitialStateHeurisicSearch() override;

    virtual void print_statistics() const override;
};
}

#endif
