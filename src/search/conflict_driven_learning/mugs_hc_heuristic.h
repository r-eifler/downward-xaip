#ifndef MUGS_HC_HEURISTIC_H
#define MUGS_HC_HEURISTIC_H

#include "hc_heuristic.h"
#include "mugs_heuristic.h"

#include <vector>

namespace conflict_driven_learning {
namespace mugs {

class MugsCriticalPathHeuristic : public MugsHeuristic {
public:
    MugsCriticalPathHeuristic(const options::Options& opts);
    static void add_options_to_parser(options::OptionParser& parser);

    hc_heuristic::HCHeuristic* get_underlying_heuristic() const { return hc_; }
    void sync();
    // virtual void print_evaluator_statistics() const override;

protected:
    virtual bool
    is_any_mug_reachable(const EvaluationContext& context) override;
    virtual void on_added_subgoal(const subgoal_t&) override;
    virtual void on_removed_subgoal(const subgoal_t&) override;

    bool check_for_reachable_mug_enumerative(int remaining_budget) const;

    bool check_for_reachable_mug_top_down(
        subgoal_t subgoal,
        const std::vector<unsigned>& unsat,
        unsigned idx,
        std::vector<unsigned>& disabled) const;

private:
    hc_heuristic::HCHeuristic* hc_;

    std::vector<bool> in_hard_goal_;
    std::vector<unsigned> goal_conjunctions_;
    std::vector<subgoal_t> conj_subgoals_;
    std::vector<std::vector<unsigned>> indices_;

    std::vector<unsigned> var_to_goal_idx_;
    std::vector<unsigned> goal_fact_ids_;

    subgoal_t all_goals_;
};

} // namespace mugs
} // namespace conflict_driven_learning

#endif
