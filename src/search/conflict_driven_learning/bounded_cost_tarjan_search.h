#ifndef BOUNDED_COST_TARJAN_SEARCH_H
#define BOUNDED_COST_TARJAN_SEARCH_H

#include "../search_engine.h"
#include "../per_state_information.h"
#include "../operator_id.h"
#include "../state_id.h"
#include "../global_state.h"
#include "../evaluator.h"
#include "heuristic_refiner.h"

#include <map>
#include <unordered_map>
#include <deque>
#include <set>

class PruningMethod;

namespace conflict_driven_learning {
namespace bounded_cost {

class BoundedCostTarjanSearch : public SearchEngine 
{
public:
    BoundedCostTarjanSearch(const options::Options& opts);
    virtual ~BoundedCostTarjanSearch() = default;
    virtual void print_statistics() const override;
    virtual double get_heuristic_refinement_time() const override;
    static void add_options_to_parser(options::OptionParser& parser);

protected:
    struct PerLayerData;

    virtual void initialize() override;
    virtual SearchStatus step() override;
    bool evaluate(const GlobalState& state, Evaluator* eval, int g);
    bool expand(const GlobalState& state);
    bool expand(const GlobalState& state,
                PerLayerData* layer);
    // bool increment_bound_and_push_initial_state();

    struct Locals {
        GlobalState state;
        OperatorID successor_op;
        std::map<std::pair<bool, int>, std::deque<std::pair<OperatorID, StateID> > > open;
        bool zero_layer;
        unsigned neighbors_size;
        Locals(const GlobalState& state, bool zero_layer, unsigned size);
    };

    struct ExpansionInfo {
        int index;
        int lowlink;
        ExpansionInfo();
    };

    class HashMap {
    private:
        std::unordered_map<StateID, ExpansionInfo> data;
    public:
        ExpansionInfo& operator[](const StateID& state);
        void remove(const StateID& state);
    };

    struct PerLayerData {
        int index;
        std::deque<GlobalState> stack;
        HashMap state_infos;
        PerLayerData();
    };

    const bool c_ignore_eval_dead_ends;
    // const bool c_recompute_h;
    bool c_refinement_toggle;
    bool c_compute_neighbors;
    const bool c_make_neighbors_unique;
    // const int c_learning_belt;

    int c_max_bound;
    double c_bound_step;

    std::shared_ptr<AbstractTask> m_task;
    TaskProxy m_task_proxy;

    Evaluator* m_expansion_evaluator;
    Evaluator* m_preferred;
    Evaluator* m_pruning_evaluator;
    std::set<Evaluator*> m_path_dependent_evaluators;
    std::shared_ptr<HeuristicRefiner> m_refiner;

    std::shared_ptr<PruningMethod> m_pruning_method;

    // PerStateInformation<PerStateInfo> m_state_infos;
    PerStateInformation<int> m_state_information;
    std::deque<PerLayerData> m_layers;
    PerLayerData* m_last_layer;
    std::deque<std::pair<int, GlobalState> > m_neighbors;
    std::deque<Locals> m_call_stack;
    bool m_solved;

    StateID m_last_state;
    int m_last_state_lowlink;
    EvaluationResult m_eval_result;
    int m_current_g;
};

}
}


#endif
