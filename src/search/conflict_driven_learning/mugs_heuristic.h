#ifndef MUGS_HEURISTIC_H
#define MUGS_HEURISTIC_H

#include "../heuristic.h"
#include "mugs_utils.h"

#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

namespace conflict_driven_learning {
namespace mugs {

class MugsHeuristic : public Heuristic {
public:
    MugsHeuristic(const options::Options& opts);
    virtual void print_statistics() const override;
    virtual EvaluationResult
    compute_result(EvaluationContext& context) override;
    static void add_options_to_parser(options::OptionParser& parser);

    const subgoal_t& get_hard_goal() const { return hard_goal_; }

    const std::vector<std::pair<int, int>>& get_goal_facts() const
    {
        return goal_assignment_;
    }

    const std::vector<std::string>& get_goal_fact_names() const
    {
        return goal_fact_names_;
    }

    const SubgoalSet& get_maximal_achieved_subgoals() const
    {
        return max_achieved_subgoals_;
    }

    std::unordered_set<subgoal_t> get_mugs_set() const
    {
        return max_achieved_subgoals_.get_minimal_extensions(
            goal_assignment_.size());
    }

    bool is_achieved(const subgoal_t& sg) const
    {
        return max_achieved_subgoals_.contains_superset(sg);
    }

    void get_goal_facts(
        const subgoal_t& sg,
        std::vector<std::pair<int, int>>& sat) const;

protected:
    virtual int compute_heuristic(const State&) override { return 0; }
    virtual bool is_any_mug_reachable(const EvaluationContext& context) = 0;
    virtual void on_removed_subgoal(const subgoal_t&) {}
    virtual void on_added_subgoal(const subgoal_t&) {}

    const int cost_bound_;
    const bool is_cost_bounded_;
private:
    const bool mugs_based_pruning_;

    subgoal_t hard_goal_;
    std::vector<std::pair<int, int>> goal_assignment_;
    std::vector<std::string> goal_fact_names_;
    SubgoalSet max_achieved_subgoals_;
    std::function<void(const subgoal_t&)> on_removed_;
};

} // namespace mugs
} // namespace conflict_driven_learning

#endif
