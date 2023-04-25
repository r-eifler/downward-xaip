#include "mugs_uc_refiner.h"

#include "../abstract_task.h"
#include "../evaluation_context.h"
#include "../evaluation_result.h"
#include "../task_proxy.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../tasks/root_task.h"
#include "mugs_utils.h"
#include "state_component.h"
#include "strips_compilation.h"

#include <unordered_set>
#include <limits>

namespace conflict_driven_learning {
namespace mugs {

MugsUCRefiner::MugsUCRefiner(const options::Options& opts)
    : mugs_hc_(std::dynamic_pointer_cast<MugsCriticalPathHeuristic>(
        opts.get<std::shared_ptr<Evaluator>>("mugs_hc")))
    , hc_refiner_(std::dynamic_pointer_cast<conflict_driven_learning::hc_heuristic::HCHeuristicRefiner>(opts.get<std::shared_ptr<HeuristicRefiner>>("hc_refiner")))
{
}

void
MugsUCRefiner::add_options_to_parser(options::OptionParser& parser)
{
    parser.add_option<std::shared_ptr<Evaluator>>("mugs_hc");
    parser.add_option<std::shared_ptr<HeuristicRefiner>>("hc_refiner");
}

Evaluator*
MugsUCRefiner::get_underlying_heuristic()
{
    return mugs_hc_.get();
}

bool
MugsUCRefiner::requires_recognized_neighbors() const
{
    return hc_refiner_->requires_neighbors();
}

void
MugsUCRefiner::print_statistics() const
{
    hc_refiner_->print_statistics();
}

void
MugsUCRefiner::initialize()
{
}

bool
MugsUCRefiner::learn_from_dead_end_component(
    StateComponent& states,
    StateComponent& neighbors_)
{
    std::unordered_set<subgoal_t> mugs = mugs_hc_->get_mugs();
    assert(!mugs.empty());

    const int bound = std::numeric_limits<int>::max();
    std::vector<std::pair<int, State>> neighbors;
    neighbors_.reset();
    while (!neighbors_.end()) {
        neighbors.emplace_back(0, neighbors_.current());
        neighbors_.next();
    }
    auto nit = SuccessorComponentIterator<std::vector<std::pair<int, State>>::const_iterator>(
            neighbors.begin(), neighbors.end());

    states.reset();
    neighbors_.reset();
    nit.reset();

    hc_refiner_->mugs_start_refinement(bound, states, nit);

    bool res = true;
    for (auto it = mugs.begin(); res && it != mugs.end(); it++) {
        subgoal_t mug = mugs_hc_->get_hard_goal() | *it;

        std::vector<std::pair<int, int>> facts;
        mugs_hc_->get_goal_facts(mug, facts);
        // mugs_hc_->get_underlying_heuristic()->set_auxiliary_goal(
        //     std::move(facts));
        // res = hc_refiner_->notify(bound, states, neighbors);
    
        hc_refiner_->mugs_refine(bound, facts, states, nit);

        states.reset();
        nit.reset();
    }
    mugs_hc_->sync();

    hc_refiner_->mugs_cleanup_after_refinement();

    return res;
}

static std::shared_ptr<ConflictLearner>
_parse(options::OptionParser& p)
{
    MugsUCRefiner::add_options_to_parser(p);
    options::Options opts = p.parse();
    if (p.dry_run()) {
        return nullptr;
    }
    return std::make_shared<MugsUCRefiner>(opts);
}

static Plugin<ConflictLearner> _plugin("mugs_uc_refiner", _parse);

} // namespace mugs
} // namespace conflict_driven_learning
