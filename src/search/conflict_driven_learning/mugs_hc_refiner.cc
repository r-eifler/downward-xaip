#include "mugs_hc_refiner.h"

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

namespace conflict_driven_learning {
namespace mugs {

MugsHCRefiner::MugsHCRefiner(const options::Options& opts)
    : mugs_hc_(dynamic_cast<MugsCriticalPathHeuristic*>(
        opts.get<Evaluator*>("mugs_hc")))
    , hc_refiner_(std::dynamic_pointer_cast<conflict_driven_learning::hc_heuristic::HCHeuristicRefiner>(opts.get<std::shared_ptr<HeuristicRefiner>>("hc_refiner")))
{
}

void
MugsHCRefiner::add_options_to_parser(options::OptionParser& parser)
{
    parser.add_option<Evaluator*>("mugs_hc");
    parser.add_option<std::shared_ptr<HeuristicRefiner>>("hc_refiner");
}

std::shared_ptr<Evaluator>
MugsHCRefiner::get_underlying_heuristic()
{
    return std::shared_ptr<Evaluator>(mugs_hc_, [](const auto&) {});
}

bool
MugsHCRefiner::requires_neighbors() const
{
    return hc_refiner_->requires_neighbors();
}

void
MugsHCRefiner::print_statistics() const
{
    hc_refiner_->print_statistics();
}

void
MugsHCRefiner::initialize()
{
}

bool
MugsHCRefiner::refine_heuristic(
    int bound,
    StateComponent& states,
    SuccessorComponent& neighbors)
{
    std::unordered_set<subgoal_t> mugs = mugs_hc_->get_mugs();
    assert(!mugs.empty());

    hc_refiner_->mugs_start_refinement(bound, states, neighbors);

    bool res = true;
    for (auto it = mugs.begin(); res && it != mugs.end(); it++) {
        subgoal_t mug = mugs_hc_->get_hard_goal() | *it;

        std::vector<std::pair<int, int>> facts;
        mugs_hc_->get_goal_facts(mug, facts);
        // mugs_hc_->get_underlying_heuristic()->set_auxiliary_goal(
        //     std::move(facts));
        // res = hc_refiner_->notify(bound, states, neighbors);
    
        hc_refiner_->mugs_refine(bound, facts, states, neighbors);

        states.reset();
        neighbors.reset();
    }
    mugs_hc_->sync();

    hc_refiner_->mugs_cleanup_after_refinement();

    return res;
}

static std::shared_ptr<HeuristicRefiner>
_parse(options::OptionParser& p)
{
    MugsHCRefiner::add_options_to_parser(p);
    options::Options opts = p.parse();
    if (p.dry_run()) {
        return nullptr;
    }
    return std::make_shared<MugsHCRefiner>(opts);
}

static Plugin<HeuristicRefiner> _plugin("mugs_hc_refiner", _parse);

} // namespace mugs
} // namespace conflict_driven_learning

