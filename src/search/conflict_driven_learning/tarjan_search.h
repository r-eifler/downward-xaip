#ifndef TARJAN_SEARCH_H
#define TARJAN_SEARCH_H

#include "../search_engine.h"
#include "../evaluator.h"
#include "../global_state.h"

#include "search_node.h"
#include "search_node_info.h"
#include "search_space.h"
#include "layered_map.h"
#include "conflict_learner.h"

#include <memory>
#include <vector>
#include <deque>
#include <set>

class PruningMethod;

namespace conflict_driven_learning
{
namespace tarjan_search
{

using SearchSpace = SearchSpaceBase<SearchNodeInfo, SearchNode>;

class TarjanSearch : public SearchEngine
{
public:
    TarjanSearch(const options::Options &opts);
    virtual void print_statistics() const override;
    virtual double get_heuristic_refinement_time() const override;
    static void add_options_to_parser(options::OptionParser &parser);
protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

    bool expand(const GlobalState &state);
    bool evaluate(const GlobalState& state);
    bool evaluate(const GlobalState& state, Evaluator* eval);
    bool evaluate_dead_end_heuristic(const GlobalState& state);
    int get_h_value() const;

    enum class DFSResult {
        FAILED = 0,
        SCC_COMPLETED = 1,
        UNRECOGNIZED = 2,
        DEAD_END_COMPONENT = 3,
        SOLVED = 4,
    };

    struct CallStackElement {
        SearchNode node;
        StateID last_successor_id;
        DFSResult succ_result;
        CallStackElement(SearchNode &node);
    };

    const bool c_recompute_u;
    const bool c_refine_initial_state;
    const bool c_prune_eval_dead_ends;
    const bool c_compatible_pruning_method;
    bool c_dead_end_refinement;
    bool c_compute_recognized_neighbors;

    EvaluationResult m_eval_result;
    Evaluator* m_guidance;
    Evaluator* m_preferred;
    std::set<Evaluator*> m_path_dependent_evaluators;
    std::shared_ptr<ConflictLearner> m_learner;
    Evaluator* m_dead_end_identifier;

    std::shared_ptr<PruningMethod> m_pruning_method;

    SearchSpace m_search_space;

    unsigned m_current_index;
    std::deque<GlobalState> m_stack;

    DFSResult m_result;
    std::deque<CallStackElement> m_call_stack;
    LayeredMultiValueMap<std::pair<bool, int>, StateID> m_open_list;

    std::deque<StateID> m_recognized_neighbors;
    std::deque<unsigned> m_rn_offset;

    size_t m_open_states;
    int m_current_depth;
};

}
}

#endif
