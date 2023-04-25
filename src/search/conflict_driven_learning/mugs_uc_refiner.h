#ifndef MUGS_HC_CONFLICT_REFINER_H
#define MUGS_HC_CONFLICT_REFINER_H

#include "conflict_learner.h"
#include "mugs_hc_heuristic.h"
#include "hc_heuristic_refiner.h"

#include <memory>

namespace options {
class Options;
class OptionParser;
} // namespace options

namespace conflict_driven_learning {
namespace mugs {

class MugsUCRefiner : public ConflictLearner {
public:
    MugsUCRefiner(const options::Options& opts);
    static void add_options_to_parser(options::OptionParser& parser);

    virtual Evaluator* get_underlying_heuristic() override;
    virtual bool requires_recognized_neighbors() const override;
    virtual void print_statistics() const override;

protected:
    virtual bool learn_from_dead_end_component(
        StateComponent& states,
        StateComponent& neighbors) override;
    virtual void initialize() override;

    std::shared_ptr<MugsCriticalPathHeuristic> mugs_hc_;
    std::shared_ptr<hc_heuristic::HCHeuristicRefiner> hc_refiner_;
};

} // namespace mugs
} // namespace conflict_driven_learning

#endif
