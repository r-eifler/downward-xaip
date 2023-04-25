#ifndef DFS_SEARCH_H
#define DFS_SEARCH_H

#include "../search_engine.h"
#include "../evaluator.h"
#include "../task_proxy.h"

#include "../../conflict_driven_learning/search_node.h"
#include "../../conflict_driven_learning/search_node_info.h"
#include "../../conflict_driven_learning/search_space.h"
#include "../../conflict_driven_learning/layered_map.h"

#include <memory>
#include <vector>
#include <deque>
#include <set>

class PruningMethod;

namespace dfs_search
{

using SearchSpace = conflict_driven_learning::SearchSpaceBase<conflict_driven_learning::tarjan_search::SearchNodeInfo, conflict_driven_learning::tarjan_search::SearchNode>;

class DepthFirstSearchSearch : public SearchEngine
{
public:
    DepthFirstSearchSearch(const options::Options &opts);
    virtual void print_statistics() const override;
    static void add_options_to_parser(options::OptionParser &parser);
protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

    bool expand(const State &state);
    bool evaluate(const State& state);
    bool evaluate(const State& state, std::shared_ptr<Evaluator> eval);
    bool evaluate_dead_end_heuristic(const State& state);
    int get_h_value() const;

    enum class DFSResult {
        FAILED = 0,
        SCC_COMPLETED = 1,
        UNRECOGNIZED = 2,
        DEAD_END_COMPONENT = 3,
        SOLVED = 4,
    };

    struct CallStackElement {
        conflict_driven_learning::tarjan_search::SearchNode node;
        StateID last_successor_id;
        DFSResult succ_result;
        CallStackElement(conflict_driven_learning::tarjan_search::SearchNode &node);
    };

    EvaluationResult m_eval_result;

    std::shared_ptr<Evaluator> m_guidance;
    std::shared_ptr<Evaluator> m_dead_end_identifier;
    std::shared_ptr<PruningMethod> m_pruning_method;

    SearchSpace m_search_space;

    unsigned m_current_index;
    std::deque<State> m_stack;

    DFSResult m_result;
    std::deque<CallStackElement> m_call_stack;
    conflict_driven_learning::LayeredMultiValueMap<int, StateID> m_open_list;

    size_t m_open_states;
    int m_current_depth;
};

}

#endif
