#ifndef QUANTITATIVE_STATE_MINIMIZATION_HC_NOGOODS_H
#define QUANTITATIVE_STATE_MINIMIZATION_HC_NOGOODS_H

#include "hc_heuristic.h"
#include "formula.h"
#include "../algorithms/segmented_vector.h"

#include <vector>

namespace conflict_driven_learning
{
namespace hc_heuristic
{

class QuantitativeStateMinimizationNoGoods : public NoGoodFormula
{
protected:
    using Formula = CounterBasedFormula;
    Formula m_formula;
    std::vector<std::map<unsigned, int> > m_clause_to_goal_cost;
    std::vector<int> m_clause_value;

    std::vector<unsigned> m_full_goal_facts;
    std::vector<unsigned> m_full_goal_conjunction_ids;

    std::vector<int> m_cur_goal_assignment;
    std::vector<std::vector<int> > m_var_orders;

    std::vector<unsigned> m_clause;
    std::vector<unsigned> m_new_facts;
    std::vector<unsigned> m_reachable_conjunctions;

    void setup_var_orders();

    virtual int evaluate_quantitative(const std::vector<unsigned> &conjunction_ids) override;
    virtual void refine_quantitative(const GlobalState &state, int bound) override;
public:
    using NoGoodFormula::NoGoodFormula;
    virtual ~QuantitativeStateMinimizationNoGoods() = default;
    virtual void initialize() override;
    virtual void synchronize_goal(std::shared_ptr<AbstractTask> task) override;
    virtual void notify_on_new_conjunction(unsigned) override;
    virtual void print_statistics() const override;
};

}
}

#endif
