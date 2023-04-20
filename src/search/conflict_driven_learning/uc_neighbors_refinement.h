#ifndef UC_NEIGHBORS_REFINEMENT_H
#define UC_NEIGHBORS_REFINEMENT_H

#include "hc_heuristic.h"
#include "hc_conflict_learner.h"
#include "../abstract_task.h"
#include "../option_parser.h"

#include <vector>
#include <deque>

namespace conflict_driven_learning
{

namespace strips {
struct Task;
}
namespace hc_heuristic
{

class UCNeighborsRefinement : public HCConflictLearner
{
    size_t m_num_refinements;
protected:
    struct OpenElement {
        const std::vector<unsigned> &conj;
        const std::vector<unsigned> &achievers;
        unsigned i;
        OpenElement(const std::vector<unsigned> &conj,
                    const std::vector<unsigned> &achievers)
            : conj(conj), achievers(achievers), i(0) {}
    };
    static const unsigned UNASSIGNED;

    const strips::Task *m_strips_task;
     // const std::shared_ptr<AbstractTask> m_task;

    // staticly initialized in each refinement
    // (but do not change within one refinement)
    size_t m_num_conjunctions_before_refinement;
    unsigned m_component_size;
    unsigned m_num_successors;

    std::vector<bool> m_current_state_unreached;
    std::vector<std::vector<unsigned> > m_fact_to_negated_component;
    std::vector<std::vector<unsigned> > m_negated_component_to_facts;
    std::vector<std::vector<unsigned> > m_conjunction_to_successors;
    std::vector<std::vector<unsigned> > m_successor_to_conjunctions;
    //// end (static)

    // dynamic data (chaning within each refinement)
    std::deque<OpenElement> m_open;
    std::vector<unsigned> m_conflict;
    std::vector<unsigned> m_regression;

    std::vector<unsigned> m_satisfied_conjunctions;
    std::vector<unsigned> m_num_covered_by;
    std::vector<unsigned> m_chosen;
    //// end (dynamic)

    void get_previously_statisfied_conjunctions(
        const std::vector<unsigned> &subgoal,
        std::vector<unsigned> &satisfied);

    unsigned get_successor_not_covers(const std::vector<unsigned> &conflict);
    bool is_subgoal_covered_by_all_successors(const std::vector<unsigned> &subgoal);
    bool is_contained_in_component(const std::vector<unsigned> &conflict);
    unsigned collect_greedy_minimal(
        const std::vector<unsigned> &superset,
        std::vector<unsigned> &num_covered,
        const std::vector<std::vector<unsigned> > &covers,
        const std::vector<std::vector<unsigned> > &covered_by,
        std::vector<unsigned> &chosen);
    void compute_conflict(const std::vector<unsigned> &subgoal,
                          std::vector<unsigned> &conflict);

    void push_conflict_for(const std::vector<unsigned> &subgoal);

    virtual bool learn_from_dead_end_component(
            StateComponent &component,
            StateComponent &rn) override;
    virtual void initialize() override;
public:
    UCNeighborsRefinement(const options::Options &opts);
    virtual bool requires_recognized_neighbors() const override;
    virtual void print_statistics() const override;
    static void add_options_to_parser(options::OptionParser &parser);
};

}
}

#endif
