#include "conflict_learner.h"

#include "../plugin.h"

namespace conflict_driven_learning
{

ConflictLearner::ConflictLearner()
{
    m_refinement_timer.stop();
    m_refinement_timer.reset();
    m_initialized = false;
}

void ConflictLearner::initialize()
{
}

void ConflictLearner::print_statistics() const
{
    std::cout << "Heuristic refinement time: "
    << m_refinement_timer << std::endl;
}

bool ConflictLearner::requires_recognized_neighbors() const
{
    return false;
}

const utils::Timer& ConflictLearner::get_refinement_timer() const
{
    return m_refinement_timer;
}

bool ConflictLearner::notify_dead_end_component(
        StateComponent &component,
        StateComponent &recognized_neighbors)
{
    if (!m_initialized) {
        m_initialized = true;
        initialize();
    }
    m_refinement_timer.resume();
    bool res = learn_from_dead_end_component(component, recognized_neighbors);
    m_refinement_timer.stop();
    return res;
}

bool ConflictLearner::notify_dead_end_component(
        StateComponent &&component,
        StateComponent &&recognized_neighbors)
{
    return notify_dead_end_component(component, recognized_neighbors);
}

}

static PluginTypePlugin<conflict_driven_learning::ConflictLearner> _plugin_type(
        "ConflictLearner", "");

