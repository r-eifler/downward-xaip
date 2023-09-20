#ifndef REACHABLE_GOALSUBSETS_CEGAR_PRUNING_H
#define REACHABLE_GOALSUBSETS_CEGAR_PRUNING_H

#include "../../heuristic.h"
#include "../../cegar/additive_cartesian_heuristic.h"
#include "msgs_collection.h"
#include "../goal_subsets/goal_subset.h"

#include <memory>

namespace new_goal_subset_heuristic {

class NewGoalSubsetHeuristic : public Heuristic {

private:

    std::vector<FactPair> soft_goal_list;
    std::vector<FactPair> hard_goal_list;
    std::vector<FactPair> all_goal_list;

    std::shared_ptr<Evaluator> h;

    std::shared_ptr<cegar::AdditiveCartesianHeuristic> cegar_heuristic;
    MSGSCollection current_msgs;

    bool initialized = false;

    goalsubset::GoalSubset get_satisfied_soft_goals(const State &state);
    goalsubset::GoalSubset get_satisfied_hard_goals(const State &state);
    goalsubset::GoalSubset get_satisfied_all_goals(const State &state);

    goalsubset::GoalSubset get_reachable_soft_goals(goalsubset::GoalSubset reachable_goals);
    goalsubset::GoalSubset get_reachable_hard_goals(goalsubset::GoalSubset reachable_goals);

public:
    explicit NewGoalSubsetHeuristic(const options::Options &opts);

    int compute_heuristic(const State &ancestor_state) override;
    StateID get_cardinally_best_state() {return current_msgs.get_cardinally_best_state();}
    int get_max_solved_soft_goals() {return current_msgs.get_max_solved_soft_goals();}
    virtual void print_statistics() const override;
    
    virtual GoalSubsets get_msgs() const;
    virtual void init_msgs(MSGSCollection goals);
};

}

#endif
