#ifndef TRAP_UNSAT_HEURISTIC_H
#define TRAP_UNSAT_HEURISTIC_H

#include "formula.h"
#include "hash_utils.h"
#include "../heuristic.h"
#include "../algorithms/segmented_vector.h"

#include <vector>
#include <functional>
#include <unordered_set>

namespace conflict_driven_learning {

class PartialStateEvaluator;

namespace strips {
struct Task;
}

namespace traps {

struct ForwardHyperTransition {
    unsigned label;
    std::vector<unsigned> destinations;
    std::vector<unsigned> progression;
    bool dead;
    ForwardHyperTransition(unsigned label)
        : label(label), dead(false)
    {}
    ForwardHyperTransition(const ForwardHyperTransition& other) = default;
};

class TrapUnsatHeuristic : public Heuristic {
public:
    using Formula = CounterBasedFormula;

    TrapUnsatHeuristic(const options::Options& opts);
    virtual ~TrapUnsatHeuristic() = default;
    virtual void set_abstract_task(std::shared_ptr<AbstractTask> task) override;

    bool for_every_regression_action(
            const std::vector<unsigned>& conj,
            std::function<bool(unsigned)> callback);
    bool for_every_progression_action(
            const std::vector<unsigned>& conj,
            std::function<bool(unsigned)> callback);
    void progression(const std::vector<unsigned>& conj,
                     unsigned op,
                     std::vector<unsigned>& post);
    bool are_dead_ends(const std::vector<unsigned>& conj);
    bool can_reach_goal(unsigned conjid) const;
    const Formula& get_all_conjunctions_formula() const;
    Formula& get_all_conjunctions_formula();
    bool evaluate_check_dead_end(const State& state);

    std::pair<unsigned, bool> insert_conjunction(const std::vector<unsigned>& conj);
    void set_transitions(unsigned conj_id, std::vector<ForwardHyperTransition>&& transitions);

    template<bool Eval = false>
    void update_reachability_insert_conjunctions();

    unsigned get_num_conjunctions() const;
    unsigned get_num_transitions() const;

    static void add_options_to_parser(options::OptionParser& parser);
protected:
    void initialize(unsigned k);
    virtual int compute_heuristic(const State &state) override;
    int compute_heuristic(const std::vector<unsigned>& state);

    bool are_mutex(const std::vector<unsigned>& conj, unsigned op) const;

    void propagate_reachability_setup_formula();

    struct HyperTransitionReference {
        unsigned label;
        unsigned idx;
        HyperTransitionReference(unsigned label, unsigned idx)
            : label(label), idx(idx) {}
        bool operator==(const HyperTransitionReference& info) const
        {
            return label == info.label && idx == info.idx;
        }
        bool operator<(const HyperTransitionReference& info) const
        {
            return label < info.label;
        }
        bool operator<(unsigned label) const
        {
            return this->label < label;
        }
    };

    struct HyperTransitionInfo {
        bool dead;
        unsigned num_out;
        unsigned counter;
        std::vector<unsigned> sources;
        HyperTransitionInfo(unsigned num_out)
            : dead(false), num_out(num_out), counter(-1) {}
        HyperTransitionInfo(const HyperTransitionInfo& other) = default;
    };

    const bool c_updatable_transitions;

    std::vector<PartialStateEvaluator*> m_evaluators;
    const conflict_driven_learning::strips::Task *m_task;

    // per action
    std::vector<std::vector<unsigned> > m_action_post;
    std::vector<std::vector<bool> > m_is_mutex;

    // per conjunction
    segmented_vector::SegmentedVector<std::vector<unsigned> > m_conjunctions;
    std::vector<bool> m_mutex_with_goal;
    std::vector<int> m_goal_reachable;
    segmented_vector::SegmentedVector<std::vector<unsigned> > m_dest_to_transition_ids;
    segmented_vector::SegmentedVector<std::vector<HyperTransitionReference> > m_transition_references;

    // per transition
    segmented_vector::SegmentedVector<std::vector<unsigned> > m_cached_progressions;
    std::unordered_set<unsigned, hash_utils::SegVecIdHash<unsigned>, hash_utils::SegVecIdEqual<unsigned> > m_progression_ids;
    segmented_vector::SegmentedVector<HyperTransitionInfo> m_transitions;
    CounterBasedFormula m_progression_lookup;

    UBTreeFormula<unsigned> m_formula;
    Formula m_formula_all;
};


template<bool Eval>
void
TrapUnsatHeuristic::update_reachability_insert_conjunctions()
{
    std::vector<unsigned> exploration_queue;
    std::vector<bool> was_unreachable(m_conjunctions.size(), false);

    for (unsigned i = 0; i < m_mutex_with_goal.size(); i++) {
        was_unreachable[i] = m_goal_reachable[i] <= 0;
        if (m_goal_reachable[i] == -1) {
            continue;
        }
        m_goal_reachable[i] = 0;
        if (!m_mutex_with_goal[i]) {
            exploration_queue.push_back(i);
            m_goal_reachable[i] = 1;
        }
    }

    for (int i = m_transitions.size() - 1; i >= 0; i--) {
        HyperTransitionInfo& info = m_transitions[i];
        info.counter = info.num_out;
        if (info.counter == 0) {
            if (Eval) {
                info.dead = are_dead_ends(m_cached_progressions[i]);
            }
            if (!info.dead) {
                for (int j = info.sources.size() - 1; j >= 0; j--) {
                    unsigned src = info.sources[j];
                    if (!m_goal_reachable[src]) {
                        m_goal_reachable[src] = 1;
                        exploration_queue.push_back(src);
                    }
                }
            }
        }
    }

    while (!exploration_queue.empty()) {
        unsigned dest = exploration_queue.back();
        exploration_queue.pop_back();
        const std::vector<unsigned>& ts = m_dest_to_transition_ids[dest];
        for (unsigned idx : ts) {
            HyperTransitionInfo& info = m_transitions[idx];
            if (--info.counter == 0) {
                if (Eval) {
                    info.dead = are_dead_ends(m_cached_progressions[idx]);
                }
                if (!info.dead) {
                    for (int j = info.sources.size() - 1; j >= 0; j--) {
                        unsigned src = info.sources[j];
                        if (!m_goal_reachable[src]) {
                            m_goal_reachable[src] = 1;
                            exploration_queue.push_back(src);
                        }
                    }
                }
            }
        }
    }

    for (unsigned i = 0; i < m_conjunctions.size(); i++) {
        if (!was_unreachable[i] && m_goal_reachable[i] <= 0) {
            m_formula.insert(m_conjunctions[i]);
        }
    }
}

}
}

#endif
