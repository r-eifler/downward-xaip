#include "mugs_hc_heuristic.h"

#include "../evaluation_context.h"
#include "../task_proxy.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "strips_compilation.h"

namespace conflict_driven_learning {
namespace mugs {

MugsCriticalPathHeuristic::MugsCriticalPathHeuristic(
    const options::Options& opts)
    : MugsHeuristic(opts)
    , hc_(std::dynamic_pointer_cast<hc_heuristic::HCHeuristic>(opts.get<std::shared_ptr<Evaluator>>("hc")))
{
    std::cout << "Initializing MugsCriticalPathHeuristic ..." << std::endl;
    all_goals_ = 0U;
    var_to_goal_idx_.resize(task->get_num_variables(), -1);
    const auto& goal = get_goal_facts();
    for (int i = goal.size() - 1; i >= 0; i--) {
        all_goals_ = all_goals_ | (1U << i);
        var_to_goal_idx_[goal[i].first] = i;
        goal_fact_ids_.push_back(strips::get_fact_id(goal[i]));
    }
    std::sort(goal_fact_ids_.begin(), goal_fact_ids_.end());
    sync();
}

void
MugsCriticalPathHeuristic::add_options_to_parser(options::OptionParser& parser)
{
    parser.add_option<std::shared_ptr<Evaluator>>("hc", "", "hc(nogoods=false)");
    MugsHeuristic::add_options_to_parser(parser);
}

hc_heuristic::HCHeuristic*
MugsCriticalPathHeuristic::get_underlying_heuristic() const
{ 
    return hc_.get();
}

void
MugsCriticalPathHeuristic::sync()
{
    goal_conjunctions_.clear();
    hc_->get_satisfied_conjunctions(goal_fact_ids_, goal_conjunctions_);

    int end = in_hard_goal_.size();
    in_hard_goal_.resize(goal_conjunctions_.size(), true);
    indices_.resize(goal_conjunctions_.size());
    conj_subgoals_.resize(goal_conjunctions_.size(), 0);
    for (int i = goal_conjunctions_.size() - 1; i >= end; i--) {
        for (const unsigned& fact_id :
             hc_->get_conjunction(goal_conjunctions_[i])) {
            int var = strips::get_variable_assignment(fact_id).first;
            if (!(get_hard_goal() & (1U << var_to_goal_idx_[var]))) {
                indices_[i].push_back(var_to_goal_idx_[var]);
            }
            conj_subgoals_[i] |= (1U << var_to_goal_idx_[var]);
        }
        in_hard_goal_[i] =
            (conj_subgoals_[i] | get_hard_goal()) == get_hard_goal();
        std::sort(indices_[i].begin(), indices_[i].end());
    }

#ifndef NDEBUG
    for (int i = goal_conjunctions_.size() - 1; i >= 0; i--) {
        subgoal_t sg = 0U;
        for (const unsigned& fact_id :
             hc_->get_conjunction(goal_conjunctions_[i])) {
            int var = strips::get_variable_assignment(fact_id).first;
            sg |= (1U << var_to_goal_idx_[var]);
        }
        assert(sg == conj_subgoals_[i]);
        assert(in_hard_goal_[i] == ((sg | get_hard_goal()) == get_hard_goal()));
    }
#endif
}

bool
MugsCriticalPathHeuristic::check_for_reachable_mug_enumerative(
    int remaining_budget) const
{
    std::vector<std::pair<int, int>> f;
    std::vector<unsigned> fids;
    std::vector<unsigned> cids;
    auto mugs = get_mugs();
    for (const subgoal_t& mug : mugs) {
        get_goal_facts(get_hard_goal() | mug, f);
        strips::get_fact_ids(fids, f);
        hc_->get_satisfied_conjunctions(fids, cids);
        bool reached = true;
        for (const unsigned& x : cids) {
            const auto& info = hc_->get_conjunction_data(x);
            if (!info.achieved()
                || (is_cost_bounded_ && info.cost >= remaining_budget)) {
                reached = false;
                break;
            }
        }
        if (reached) {
            return true;
        }
        f.clear();
        fids.clear();
        cids.clear();
    }
    return false;
}

bool
MugsCriticalPathHeuristic::is_any_mug_reachable(
    const EvaluationContext& context)
{
    int remaining_budget = cost_bound_ - context.get_g_value();
    std::vector<unsigned> state_fact_ids;
    strips::get_fact_ids(state_fact_ids, context.get_state());
    int hval = hc_->compute_heuristic_for_facts(state_fact_ids);
    // NOTE: for the following check to make sense, the heuristic must consider
    // the entire goal (including all soft-goals)
    if (hval != DEAD_END && (!is_cost_bounded_ || hval < remaining_budget)) {
        return true;
    }
    std::vector<unsigned> tobeprocessed;
    for (unsigned i = 0; i < goal_conjunctions_.size(); i++) {
        const auto& info = hc_->get_conjunction_data(goal_conjunctions_[i]);
        if (!info.achieved()
            || (is_cost_bounded_ && info.cost >= remaining_budget)) {
            if (in_hard_goal_[i]) {
                // hard goals cannot be removed
                assert(!check_for_reachable_mug_enumerative(remaining_budget));
                return false;
            }
            tobeprocessed.push_back(i);
        }
    }
    std::vector<unsigned> disabled(get_goal_facts().size(), 0);
    bool result = check_for_reachable_mug_top_down(
        all_goals_, tobeprocessed, 0, disabled);
    assert(result == check_for_reachable_mug_enumerative(remaining_budget));
    return result;
}

void
MugsCriticalPathHeuristic::on_added_subgoal(const subgoal_t&)
{
}

void
MugsCriticalPathHeuristic::on_removed_subgoal(const subgoal_t&)
{
}

bool
MugsCriticalPathHeuristic::check_for_reachable_mug_top_down(
    subgoal_t subgoal,
    const std::vector<unsigned>& unsat,
    unsigned i,
    std::vector<unsigned>& disabled) const
{
    unsigned idx = unsat[i];
    bool all_done = (i + 1) == unsat.size();
    if ((subgoal & conj_subgoals_[idx]) != conj_subgoals_[idx]) {
        return all_done
            ? true
            : check_for_reachable_mug_top_down(subgoal, unsat, i + 1, disabled);
    }
    for (unsigned j = 0; j < indices_[idx].size(); j++) {
        unsigned x = indices_[idx][j];
        if (disabled[x]++ == 0) {
            subgoal_t sg = subgoal & ~(1U << x);
            if (!is_achieved(sg)
                && (all_done
                    || check_for_reachable_mug_top_down(
                        sg, unsat, i + 1, disabled))) {
                return true;
            }
        }
    }
    for (unsigned j = 0; j < indices_[idx].size(); j++) {
        unsigned x = indices_[idx][j];
        disabled[x]--;
    }
    return false;
}

// void 
// MugsCriticalPathHeuristic::print_evaluator_statistics() const
// {
//     hc_->print_statistics();
//     MugsHeuristic::print_evaluator_statistics();
// }

static std::shared_ptr<Heuristic> _parse(OptionParser &parser) {

    MugsCriticalPathHeuristic::add_options_to_parser(parser);
    options::Options opts = parser.parse();
    opts.set<bool>("cache_estimates", false);
    if (parser.dry_run()) {
        return nullptr;
    }
    return std::make_shared<MugsCriticalPathHeuristic>(opts);
}

static Plugin<Evaluator> _plugin("mugs_hc", _parse);

} // namespace mugs
} // namespace conflict_driven_learning

