#ifndef MUGS_HC_GENERAL_HEURISTIC_REFINER_H
#define MUGS_HC_GENERAL_HEURISTIC_REFINER_H

#include "heuristic_refiner.h"
#include "mugs_hc_heuristic.h"
#include "hc_heuristic_refiner.h"

#include <memory>

namespace options {
class Options;
class OptionParser;
} // namespace options

namespace conflict_driven_learning {
namespace mugs {

class MugsHCRefiner : public HeuristicRefiner {
public:
    MugsHCRefiner(const options::Options& opts);
    static void add_options_to_parser(options::OptionParser& parser);

    virtual std::shared_ptr<Evaluator> get_underlying_heuristic() override;
    virtual bool requires_neighbors() const override;
    virtual void print_statistics() const override;

protected:
    virtual bool refine_heuristic(int bound,
                                  StateComponent& component,
                                  SuccessorComponent& neighbors) override;
    virtual void initialize() override;

    std::shared_ptr<MugsCriticalPathHeuristic> mugs_hc_;
    std::shared_ptr<hc_heuristic::HCHeuristicRefiner> hc_refiner_;
};

} // namespace mugs
} // namespace conflict_driven_learning

#endif
