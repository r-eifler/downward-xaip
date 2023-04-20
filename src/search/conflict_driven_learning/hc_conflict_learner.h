#ifndef HC_CONFLICT_LEARNER_H
#define HC_CONFLICT_LEARNER_H

#include "hc_heuristic.h"
#include "conflict_learner.h"
#include "../option_parser.h"

#include <memory>

namespace conflict_driven_learning
{
namespace hc_heuristic
{

class HCConflictLearner : public ConflictLearner
{
protected:
    const float c_x_limit;
    size_t c_counter_limit;
    std::shared_ptr<HCHeuristic> m_hc;
public:
    HCConflictLearner(const options::Options &opts);
    bool size_limit_reached() const;
    virtual Evaluator* get_underlying_heuristic() override;
    static void add_options_to_parser(options::OptionParser &parser);
};

}
}

#endif
