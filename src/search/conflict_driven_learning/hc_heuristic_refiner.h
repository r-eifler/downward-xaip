#ifndef HC_HEURISTIC_REFINER_H
#define HC_HEURISTIC_REFINER_H

#include "../option_parser.h"
#include "hc_heuristic.h"
#include "heuristic_refiner.h"

#include <memory>

namespace conflict_driven_learning {
namespace hc_heuristic {

class HCHeuristicRefiner : public HeuristicRefiner {
protected:
    const float c_x_limit;
    size_t c_counter_limit;
    std::shared_ptr<HCHeuristic> m_hc;

public:
    HCHeuristicRefiner(const options::Options& opts);
    bool size_limit_reached() const;

    virtual void
    mugs_start_refinement(int , StateComponent&, SuccessorComponent&)
    {
    }
    virtual bool mugs_refine(
        int bound,
        const std::vector<std::pair<int, int>>& mugs,
        StateComponent& component,
        SuccessorComponent&) = 0;
    virtual void mugs_cleanup_after_refinement() {}

    virtual std::shared_ptr<Evaluator> get_underlying_heuristic() override;
    static void add_options_to_parser(options::OptionParser& parser);
};

} // namespace hc_heuristic
} // namespace conflict_driven_learning

#endif
