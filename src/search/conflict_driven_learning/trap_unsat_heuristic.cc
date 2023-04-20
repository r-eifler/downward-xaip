#include "trap_unsat_heuristic.h"
#include "strips_compilation.h"
#include "partial_state_evaluator.h"
#include "set_utils.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../global_state.h"
#include "../utils/timer.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <cstdio>
#include <set>

namespace conflict_driven_learning {
namespace traps {

TrapUnsatHeuristic::TrapUnsatHeuristic(const options::Options& opts)
    : Heuristic(opts)
    , c_updatable_transitions(opts.get<bool>("update_transitions"))
    , m_progression_ids(
            10000,
            hash_utils::SegVecIdHash<unsigned>(m_cached_progressions),
            hash_utils::SegVecIdEqual<unsigned>(m_cached_progressions))
{
    strips::initialize(*task);
    m_task = &strips::get_task();
    m_formula.set_num_keys(strips::num_facts());
    m_formula_all.set_num_keys(strips::num_facts());
    if (c_updatable_transitions) {
        m_progression_lookup.set_num_keys(strips::num_facts());
    }
    std::vector<Evaluator*> evals = opts.get_list<Evaluator*>("evals");
    for (unsigned i = 0; i < evals.size(); i++) {
        m_evaluators.push_back(dynamic_cast<PartialStateEvaluator*>(evals[i]));
        assert(m_evaluators.back() != nullptr);
    }
    initialize(opts.get<int>("k"));
}

void
TrapUnsatHeuristic::add_options_to_parser(options::OptionParser& parser)
{
    parser.add_option<int>("k", "", "0");
    parser.add_list_option<Evaluator *>("evals", "", "[]");
    parser.add_option<bool>("update_transitions", "", "false");
    Heuristic::add_options_to_parser(parser);
}

void
TrapUnsatHeuristic::initialize(unsigned k)
{
    std::cout << "Initializating traps heuristic ..." << std::endl;
    utils::Timer initialiation_t;

    m_action_post.resize(m_task->num_actions());
    for (unsigned op = 0; op < m_action_post.size(); op++) {
        const strips::Action &action = m_task->get_action(op);
        std::set_union(action.pre.begin(), action.pre.end(),
                       action.add.begin(), action.add.end(),
                       std::back_inserter(m_action_post[op]));
        set_utils::inplace_difference(m_action_post[op], action.del);
        assert(std::is_sorted(m_action_post[op].begin(), m_action_post[op].end()));
        assert(std::unique(m_action_post[op].begin(),
                           m_action_post[op].end()) == m_action_post[op].end());
        assert(!m_task->contains_mutex(m_action_post[op]));
    }

    m_is_mutex.resize(m_task->num_actions());
    for (unsigned i = 0; i < m_is_mutex.size(); i++) {
        m_is_mutex[i].resize(strips::num_facts(), false);
        const strips::Action &action = m_task->get_action(i);
        for (unsigned q = 0; q < m_is_mutex[i].size(); q++) {
            m_is_mutex[i][q] = m_task->are_mutex(q, action.pre) ||
                               (!set_utils::contains(action.del, q) && m_task->are_mutex(q, action.add));
        }
    }

    if (k > 0) {
        std::cout << "Generating k<=" << k << " conjunctions (t=" << initialiation_t << ") ..." << std::endl;
        std::vector<bool> fact_mutex_with_goal(strips::num_facts(), false);
        for (unsigned p = 0; p < fact_mutex_with_goal.size(); p++) {
            fact_mutex_with_goal[p] = m_task->are_mutex(p, m_task->get_goal());
        }
        std::vector<bool> in_goal(task->get_num_variables(), false);
        for (int i = 0; i < task->get_num_goals(); i++) {
            in_goal[task->get_goal_fact(i).var] = true;
        }

        unsigned num_mutex = 0;
        struct EnumData {
            int var;
            int succ_var;
            int val;
            bool goalvar;
            bool is_goal_mutex;
            EnumData(int var, int domain, bool goalvar, bool mut)
                : var(var), succ_var(var), val(domain), goalvar(goalvar), is_goal_mutex(mut)
            {}
        };
        std::vector<EnumData> vars;
        std::vector<unsigned> fact_ids;
        for (int var = 0; var < task->get_num_variables(); var++) {
            vars.emplace_back(var, task->get_variable_domain_size(var), in_goal[var], false);
            fact_ids.push_back(-1);
            while (!vars.empty()) {
                EnumData& e = vars.back();
                fact_ids.pop_back();
                if (--e.val < 0) {
                    vars.pop_back();
                    continue;
                }
                unsigned fact_id = strips::get_fact_id(e.var, e.val);
                bool goal_mutex = fact_mutex_with_goal[fact_id];
                fact_ids.push_back(fact_id);
                if (e.goalvar) {
                    m_conjunctions.push_back(fact_ids);
                    m_formula_all.insert(fact_ids);
                    m_mutex_with_goal.push_back(e.is_goal_mutex || goal_mutex);
                    if (m_mutex_with_goal.back()) { num_mutex++; }
                }
                if (fact_ids.size() < k
                        && ++e.succ_var < task->get_num_variables()) {
                    vars.emplace_back(
                            e.succ_var,
                            task->get_variable_domain_size(e.succ_var),
                            e.goalvar || in_goal[e.succ_var],
                            e.is_goal_mutex || goal_mutex);
                    fact_ids.push_back(-1);
                }
            }
            assert(fact_ids.empty());
        }
        std::cout << "Generated " << m_conjunctions.size() << " conjunctions, "
            << num_mutex << " are mutex with the goal" << std::endl;

        std::cout << "Computing transition relation (t=" << initialiation_t
            << ") ..." << std::endl;
        m_dest_to_transition_ids.resize(m_conjunctions.size());
        m_transition_references.resize(m_conjunctions.size());
        unsigned num_transitions = 0;
        for (unsigned i = 0; i < m_conjunctions.size(); i++) {
            const std::vector<unsigned>& conj = m_conjunctions[i];
            std::vector<HyperTransitionReference>& trefs = m_transition_references[i];
            for_every_progression_action(conj, [&](unsigned op) {
                m_cached_progressions.push_back(std::vector<unsigned>());
                std::vector<unsigned>& prog = m_cached_progressions[num_transitions];
                progression(conj, op, prog);
                assert(!m_task->contains_mutex(prog));
                
                auto trans_id = m_progression_ids.insert(num_transitions);
                if (trans_id.second) {
                    unsigned num = 0;
                    m_formula_all.forall_subsets(prog, [&](unsigned id) {
                        m_dest_to_transition_ids[id].push_back(num_transitions);
                        num++;
                        return false;
                    });
                    m_transitions.push_back(HyperTransitionInfo(num));
                    num_transitions++;
                    if (c_updatable_transitions) {
                        m_progression_lookup.insert(prog);
                    }
                } else {
                    m_cached_progressions.resize(num_transitions);
                }

                trefs.emplace_back(op, *trans_id.first);
                m_transitions[*trans_id.first].sources.push_back(i);

                return false;
            });
            std::sort(trefs.begin(), trefs.end());
        }
        std::cout << "Generated " << num_transitions << " transitions" << std::endl;

        std::cout << "Performing reachability analysis (t=" << initialiation_t << ")" << std::endl;
        propagate_reachability_setup_formula();
    }

    std::cout << "Initialized trap with " << m_formula.size()
              << " conjunctions after " << initialiation_t
              << std::endl;
}

void
TrapUnsatHeuristic::set_abstract_task(std::shared_ptr<AbstractTask> task)
{
    // assumes that abstract task is updated for all evaluators beforehand
    strips::update_goal_set(*task);
    std::fill(m_mutex_with_goal.begin(), m_mutex_with_goal.end(), false);
    std::vector<bool> fact_mutex_with_goal(strips::num_facts(), false);
    for (unsigned p = 0; p < fact_mutex_with_goal.size(); p++) {
        fact_mutex_with_goal[p] = m_task->are_mutex(p, m_task->get_goal());
    }
    for (int i = m_conjunctions.size() - 1; i >= 0; i--) {
        const std::vector<unsigned>& conj = m_conjunctions[i];
        for (int j = conj.size() - 1; j >= 0; j--) {
            if (fact_mutex_with_goal[conj[j]]) {
                m_mutex_with_goal[i] = true;
                break;
            }
        }
    }
    m_formula.clear();
    propagate_reachability_setup_formula();
}

int
TrapUnsatHeuristic::compute_heuristic(const std::vector<unsigned> &state)
{
    if (m_formula.contains_subset_of(state)) {
        return DEAD_END;
    }
    return 0;
}

int
TrapUnsatHeuristic::compute_heuristic(const GlobalState &state)
{
    static std::vector<unsigned> state_fact_ids;
    state_fact_ids.clear();
    for (int var = 0; var < task->get_num_variables(); var++) {
        state_fact_ids.push_back(strips::get_fact_id(var, state[var]));
    }
    return compute_heuristic(state_fact_ids);
}

bool
TrapUnsatHeuristic::are_dead_ends(const std::vector<unsigned> &phi)
{
    if (m_evaluators.empty()) {
        return false;
    }
    std::vector<std::pair<int, int> > partial_state;
    partial_state.reserve(phi.size());
    for (const unsigned &p : phi) {
        partial_state.push_back(strips::get_variable_assignment(p));
    }
    bool res = false;
    for (unsigned i = 0; i < m_evaluators.size(); i++) {
        if (m_evaluators[i]->evaluate_partial_state(partial_state) == DEAD_END) {
            res = true;
            break;
        }
    }
    return res;
}

void
TrapUnsatHeuristic::progression(const std::vector<unsigned> &phi,
                                unsigned op,
                                std::vector<unsigned> &post)
{
    assert(op < m_task->num_actions());
    assert(op < m_action_post.size());
    const strips::Action &action = m_task->get_action(op);
    std::set_difference(phi.begin(), phi.end(),
                        action.del.begin(), action.del.end(),
                        std::back_inserter(post));
    set_utils::inplace_union(post, m_action_post[op]);
    assert(!m_task->contains_mutex(post));
    assert(phi != post);
}

bool
TrapUnsatHeuristic::are_mutex(const std::vector<unsigned> &phi,
                                   unsigned op) const
{
    assert(op < m_is_mutex.size());
    const std::vector<bool> &mutex = m_is_mutex[op];
    for (const unsigned &p : phi) {
        assert(p < mutex.size());
        if (mutex[p]) {
            return true;
        }
    }
    return false;
}

bool
TrapUnsatHeuristic::for_every_progression_action(
        const std::vector<unsigned>& phi,
        std::function<bool(unsigned)> callback)
{
    static std::vector<bool> closed;
    closed.resize(m_task->num_actions());
    std::fill(closed.begin(), closed.end(), false);
    bool skip = false;
    for (const unsigned& p : phi) {
        const std::vector<unsigned>& ops = m_task->get_actions_with_del(p);
        for (const unsigned& op : ops) {
            if (!closed[op]) {
                closed[op] = true;
                if (!are_mutex(phi, op)) {
                    if (callback(op)) {
                        skip = true;
                        break;
                    }
                }
            }
        }
        if (skip) {
            break;
        }
    }
    return skip;
}

bool
TrapUnsatHeuristic::for_every_regression_action(
        const std::vector<unsigned>& conj,
        std::function<bool(unsigned)> callback)
{
    static std::vector<bool> closed;
    closed.resize(m_task->num_actions());
    std::fill(closed.begin(), closed.end(), false);
    for (const unsigned& p : conj) {
        const std::vector<unsigned>& ops = m_task->get_actions_with_del(p);
        for (const unsigned& op : ops) {
            closed[op] = true;
        }
    }
    for (const unsigned& p : conj) {
        const std::vector<unsigned>& ops = m_task->get_actions_with_add(p);
        bool done = false;
        for (const unsigned& op : ops) {
            if (!closed[op]) {
                closed[op] = true;
                if (callback(op)) {
                    done = true;
                    break;
                }
            }
        }
        if (done) {
            return true;
        }
    }
    return false;
}

void
TrapUnsatHeuristic::propagate_reachability_setup_formula()
{
    m_goal_reachable.resize(m_conjunctions.size());
    std::fill(m_goal_reachable.begin(), m_goal_reachable.end(), 1);

    if (m_evaluators.size() > 0) {
        for (int i = m_transitions.size() - 1; i >= 0; i--) {
            m_transitions[i].dead = false;
        }
        for (unsigned i = 0; i < m_conjunctions.size(); i++) {
            if (m_goal_reachable[i] == 0 && are_dead_ends(m_conjunctions[i])) {
                m_goal_reachable[i] = -1;
                m_formula.insert(m_conjunctions[i]);
            }
        }
        update_reachability_insert_conjunctions<true>();
    } else {
        update_reachability_insert_conjunctions<false>();
    }
}

TrapUnsatHeuristic::Formula&
TrapUnsatHeuristic::get_all_conjunctions_formula()
{
    return m_formula_all;
}

const TrapUnsatHeuristic::Formula&
TrapUnsatHeuristic::get_all_conjunctions_formula() const
{
    return m_formula_all;
}

bool
TrapUnsatHeuristic::can_reach_goal(unsigned conjid) const
{
    return m_goal_reachable[conjid] >= 1;
}

std::pair<unsigned, bool>
TrapUnsatHeuristic::insert_conjunction(
        const std::vector<unsigned>& conj)
{
#ifndef NDEBUG
    bool mutex = false;
    for (unsigned i = 0; i < conj.size(); i++) {
        mutex = mutex || m_task->are_mutex(conj[i], m_task->get_goal());
    }
    assert(mutex);
#endif
    unsigned conj_id = -1;
    // use hash set instead?
    m_formula_all.forall_subsets(conj, [&](unsigned id) {
        if (m_conjunctions[id].size() == conj.size()) {
            conj_id = id;
        }
        return false;
    });
    if (conj_id != (unsigned) -1) {
        assert(m_mutex_with_goal[conj_id]);
        if (m_goal_reachable[conj_id] == 1) {
            m_goal_reachable[conj_id] = 0;
            m_formula.insert(conj);
        }
        return std::pair<unsigned, bool>(conj_id, false);
    } else {
        m_conjunctions.push_back(conj);
        m_formula.insert(conj);
        m_formula_all.insert(conj);
        m_mutex_with_goal.push_back(true);
        m_goal_reachable.push_back(0);
        m_transition_references.push_back(std::vector<HyperTransitionReference>());
        m_dest_to_transition_ids.push_back(std::vector<unsigned>());
        if (c_updatable_transitions) {
            std::vector<unsigned>& ts = m_dest_to_transition_ids[m_conjunctions.size() - 1];
            m_progression_lookup.forall_supersets(conj, [&](unsigned id) {
                ts.push_back(id);
                m_transitions[id].num_out++;
            });
        }
        return std::pair<unsigned, bool>(m_conjunctions.size() - 1, true);
    }
}

void
TrapUnsatHeuristic::set_transitions(
        unsigned conj_id,
        std::vector<ForwardHyperTransition>&& transitions)
{
    if (transitions.empty()) {
        assert(m_transition_references[conj_id].empty());
        return;
    }

    std::sort(transitions.begin(), transitions.end(), [](const ForwardHyperTransition& x, const ForwardHyperTransition& y) {
            return x.label < y.label;
    });

    std::vector<HyperTransitionReference>& trefs = m_transition_references[conj_id];
    assert(trefs.empty() || trefs.size() == transitions.size());

    if (trefs.empty()) {
        unsigned num_transitions = m_cached_progressions.size();
        for (unsigned i = 0; i < transitions.size(); i++) {
            ForwardHyperTransition& t = transitions[i];
            m_cached_progressions.push_back(t.progression);
            auto insrt = m_progression_ids.insert(num_transitions);
            if (insrt.second) {
                m_transitions.push_back(HyperTransitionInfo(0));
                if (c_updatable_transitions) {
                    m_progression_lookup.insert(t.progression);
                }
                num_transitions++;
            } else {
                m_cached_progressions.resize(num_transitions);
            }
            unsigned t_idx = *insrt.first;
            HyperTransitionInfo& info = m_transitions[t_idx];
            if (info.num_out != t.destinations.size()) {
                assert(info.num_out < t.destinations.size());
                info.num_out = t.destinations.size();
                for (const unsigned& dest : t.destinations) {
                    set_utils::insert(m_dest_to_transition_ids[dest], t_idx);
                }
            }
            trefs.emplace_back(t.label, t_idx);
            info.sources.push_back(conj_id);
            info.dead = info.dead || t.dead;
        }
    } else {
        for (unsigned i = 0; i < transitions.size(); i++) {
            ForwardHyperTransition& t = transitions[i];
            assert(trefs[i].label == t.label);
            unsigned t_idx = trefs[i].idx;
            HyperTransitionInfo& info = m_transitions[t_idx];
            if (info.num_out != t.destinations.size()) {
                assert(info.num_out < t.destinations.size());
                info.num_out = t.destinations.size();
                for (const unsigned& dest : t.destinations) {
                    set_utils::insert(m_dest_to_transition_ids[dest], t_idx);
                }
            }
            info.dead = info.dead || t.dead;
        }
    }
}

bool
TrapUnsatHeuristic::evaluate_check_dead_end(const GlobalState& state)
{
    return compute_heuristic(state) == DEAD_END;
}

unsigned
TrapUnsatHeuristic::get_num_conjunctions() const
{
    return m_conjunctions.size();
}

unsigned
TrapUnsatHeuristic::get_num_transitions() const
{
    return m_transitions.size();
}
    
}
}

static Heuristic*
_parse(options::OptionParser& parser)
{
    conflict_driven_learning::traps::TrapUnsatHeuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        opts.set<bool>("cache_estimates", false);
        return new conflict_driven_learning::traps::TrapUnsatHeuristic(opts);
    }
    return nullptr;
}

static Plugin<Evaluator> _plugin("traps", _parse);
