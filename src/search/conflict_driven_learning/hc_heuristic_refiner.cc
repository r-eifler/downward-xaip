#include "hc_heuristic_refiner.h"

#include <limits>

namespace conflict_driven_learning
{
namespace hc_heuristic
{

template<typename T>
void noop_delete(T* ) {}

HCHeuristicRefiner::HCHeuristicRefiner(const options::Options &opts)
    : c_x_limit(opts.get<double>("x")),
      m_hc(dynamic_cast<HCHeuristic *>(opts.get<Evaluator *>("hc")), noop_delete<HCHeuristic>)
{
    if (c_x_limit < 0) {
        c_counter_limit = std::numeric_limits<size_t>::max();
    } else {
        c_counter_limit = c_x_limit * m_hc->num_atomic_counters();
    }
}

bool HCHeuristicRefiner::size_limit_reached() const
{
    return m_hc->num_counters() >= c_counter_limit;
}

std::shared_ptr<Evaluator> HCHeuristicRefiner::get_underlying_heuristic()
{
    assert(m_hc != nullptr);
    return m_hc;
}

void HCHeuristicRefiner::add_options_to_parser(options::OptionParser &parser)
{
    parser.add_option<Evaluator *>("hc");
    parser.add_option<double>("x", "", "-1");
}

}
}
