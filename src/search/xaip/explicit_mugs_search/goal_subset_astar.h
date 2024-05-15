#ifndef SEARCH_ENGINES_GOAL_SUBSET_ASTAR_SEARCH_H
#define SEARCH_ENGINES_GOAL_SUBSET_ASTAR_SEARCH_H

#include "../open_list.h"
#include "../search_engine.h"
#include "msgs_collection.h"
#include "msgs_evaluation_context.h"
#include "../policy/policy_pruning.h"

#include <memory>
#include <vector>

class Evaluator;
class PruningMethod;

namespace options {
class OptionParser;
class Options;
}

namespace goal_subset_astar {
class GoalSubsetAStar : public SearchEngine {
    const bool reopen_closed_nodes;
    const bool anytime;
    const int max_print_mugs;

    std::unique_ptr<StateOpenList> open_list;
    std::shared_ptr<Evaluator> f_evaluator;

    std::vector<Evaluator *> path_dependent_evaluators;
    std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
    std::shared_ptr<Evaluator> lazy_evaluator;
    std::shared_ptr<Evaluator> eval;

    std::shared_ptr<policy_pruning_method::PolicyPruningMethod> pruning_method;

    MSGSCollection current_msgs;

    void start_f_value_statistics(MSGSEvaluationContext &eval_context);
    void update_f_value_statistics(MSGSEvaluationContext &eval_context);
    void reward_progress();

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    explicit GoalSubsetAStar(const options::Options &opts);
    virtual ~GoalSubsetAStar() = default;

    // void set_pruning_method( std::shared_ptr<PruningMethod> pruning_method){
    //     std::cout << "********** set pruning method ***************" << std::endl;
    //     this->pruning_method = pruning_method;
    // }

    // std::shared_ptr<PruningMethod> get_pruning_method(){
    //     return pruning_method;
    // }

    void init_msgs(MSGSCollection collection);
    MSGSCollection get_msgs();

    virtual void print_statistics() const override;

    void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
}

#endif
