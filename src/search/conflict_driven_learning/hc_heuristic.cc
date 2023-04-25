#include "hc_heuristic.h"

#include "state_minimization_nogoods.h"
#include "quantitative_state_minimization_nogoods.h"
#include "strips_compilation.h"
#include "set_utils.h"

#include "../evaluation_context.h"
#include "../abstract_task.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../utils/timer.h"

#include "../task_utils/task_properties.h"

#include <cassert>
#include <iostream>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <fstream>

namespace conflict_driven_learning {

using namespace strips;

namespace hc_heuristic {

NoGoodFormula::NoGoodFormula(std::shared_ptr<AbstractTask> task,
                             HCHeuristic* hc)
    : m_task(task)
    , m_hc(hc)
{
    m_evaluation_timer.stop();
    m_evaluation_timer.reset();
    m_refinement_timer.stop();
    m_refinement_timer.reset();
}

bool NoGoodFormula::evaluate_formula(
    const std::vector<unsigned> &conjunction_ids)
{
    if (m_hc == NULL) {
        return false;
    }
    m_evaluation_timer.resume();
    bool res = evaluate(conjunction_ids);
    m_evaluation_timer.stop();
    return res;
}

void NoGoodFormula::refine_formula(
    const State &state,
    int bound)
{
    if (m_hc == NULL) {
        return;
    }
    m_refinement_timer.resume();
    refine_quantitative(state, bound);
    m_refinement_timer.stop();
}

int NoGoodFormula::evaluate_formula_quantitative(
    const std::vector<unsigned> &conjunction_ids)
{
    if (m_hc == NULL) {
        return false;
    }
    m_evaluation_timer.resume();
    int res = evaluate_quantitative(conjunction_ids);
    m_evaluation_timer.stop();
    return res;
}

void NoGoodFormula::refine_formula(
    const State &state)
{
    if (m_hc == NULL) {
        return;
    }
    m_refinement_timer.resume();
    refine(state);
    m_refinement_timer.stop();
}

const utils::Timer &NoGoodFormula::get_refinement_timer() const
{
    return m_refinement_timer;
}

const utils::Timer &NoGoodFormula::get_evaluation_timer() const
{
    return m_evaluation_timer;
}

const int HCHeuristic::DEAD_END = -1;

HCHeuristic::HCHeuristic(const options::Options &opts)
    : Heuristic(opts),
      c_prune_subsumed_preconditions(opts.get<bool>("prune_subsumed_preconditions")),
      c_early_termination(true),
      c_nogood_evaluation_enabled(false),
      m_hc_evaluations(0),
      cost_bound_(opts.get<int>("cost_bound")),
      m_nogood_formula(nullptr),
      store_conjunctions_("")
      // m_nogood_formula(opts.contains("nogoods") ?
      //                  opts.get<NoGoodFormula * >("nogoods") : NULL)
{
    utils::Timer timer_init;
    if (opts.get<bool>("nogoods")) {
        c_nogood_evaluation_enabled = true;
        if (cost_bound_ >= 0) {
            m_nogood_formula = std::unique_ptr<NoGoodFormula>(new QuantitativeStateMinimizationNoGoods(task, this));
        } else {
            m_nogood_formula = std::unique_ptr<NoGoodFormula>(new StateMinimizationNoGoods(task, this));
        }
        assert(m_nogood_formula != nullptr);
    }
    initialize(opts.get<int>("m"));
    if (opts.contains("conjs_in")) {
        std::unordered_map<std::string, std::pair<int, int> > lookup;
        for (int var = 0; var < task->get_num_variables(); var++) {
            for (int val = 0; val < task->get_variable_domain_size(var); val++) {
                lookup[task->get_fact_name(FactPair(var, val))] = std::make_pair(var, val);
            }
        }

        std::ifstream infile(opts.get<std::string>("conjs_in"));
        std::string line;
        std::vector<unsigned> conj;
        while (std::getline(infile, line)){
            while (true) {
                size_t j = line.find(" | ");
                if (j == std::string::npos) {
                    conj.push_back(strips::get_fact_id(lookup[line]));
                    break;
                } else {
                    conj.push_back(strips::get_fact_id(lookup[line.substr(0, j)]));
                    line = line.substr(j+3);
                }
            }
            std::sort(conj.begin(), conj.end());
            insert_conjunction_and_update_data_structures(conj);
            conj.clear();
        }
        infile.close();
    }
    if (opts.contains("conjs_out")) {
        store_conjunctions_ = opts.get<std::string>("conjs_out");
    }
    std::cout << "Initialized hC after "
        << timer_init
        << ", generated "
        << m_conjunction_data.size() << " conjunctions and "
        << m_counters.size() << " counters (ratio "
           << (((double) m_counters.size()) / m_num_atomic_counters)
           << ")"
           << std::endl;

    reset_auxiliary_goal();
}

void
HCHeuristic::set_auxiliary_goal(std::vector<std::pair<int, int> >&& aux)
{
    auxiliary_goal_conjunctions_.clear();
    auxiliary_goal_ = std::move(aux);
    assert(std::is_sorted(auxiliary_goal_.begin(), auxiliary_goal_.end()));
    std::vector<unsigned> facts;
    strips::get_fact_ids(facts, auxiliary_goal_);
    get_satisfied_conjunctions(facts, auxiliary_goal_conjunctions_);
}

void
HCHeuristic::reset_auxiliary_goal()
{
    auxiliary_goal_.clear();
    for (int i = 0; i < task->get_num_goals(); i++) {
        FactPair g = task->get_goal_fact(i);
        auxiliary_goal_.emplace_back(g.var, g.value);
    }
    auxiliary_goal_conjunctions_ = get_counter_precondition(m_goal_counter);
}

const std::vector<std::pair<int, int> >&
HCHeuristic::get_auxiliary_goal() const
{
    return auxiliary_goal_;
}

const std::vector<unsigned>&
HCHeuristic::get_auxiliary_goal_conjunctions() const
{
    return auxiliary_goal_conjunctions_;
}

void HCHeuristic::initialize(unsigned m)
{
    std::cout << "Initializing hC heuristic ..." << std::endl;
    strips::initialize(*task);


    const strips::Task &strips_task = strips::get_task();
    std::vector<std::vector<unsigned> > temp_precondition;
    std::vector<std::vector<unsigned> > temp_achievers(
        strips::num_facts());

    m_conjunctions.resize(strips::num_facts());
    m_fact_to_conjunctions.resize(strips::num_facts());
    m_conjunction_size.resize(strips::num_facts(), 1);
    m_subset_count.resize(strips::num_facts());
    for (unsigned p = 0; p < strips::num_facts(); p++) {
        m_conjunction_data.push_back(ConjunctionData(p));
        m_conjunctions[p].push_back(p);
        m_fact_to_conjunctions[p].push_back(p);
    }
    m_counters_with_fact_precondition.resize(strips::num_facts());

    m_goal_counter = 0;
    temp_precondition.push_back(strips_task.get_goal());
    for (const unsigned &p : strips_task.get_goal()) {
        m_counters_with_fact_precondition[p].push_back(m_goal_counter);
    }
    m_counter_to_action.push_back(-1);

    size_t num_counters = strips_task.generate_singleton_effect_actions(
                              temp_precondition,
                              temp_achievers,
                              m_counters_with_fact_precondition,
                              m_counter_to_action);
    m_counters.resize(num_counters);
    m_counters[m_goal_counter].effect = &m_goal_conjunction;

    for (unsigned counterid = 0; counterid < m_counters.size(); counterid++) {
        Counter &counter = m_counters[counterid];
        counter.id = counterid;
        counter.preconditions = temp_precondition[counterid].size();
        counter.action_cost = m_counter_to_action[counterid] < strips_task.num_actions()
                              ? strips_task.get_action(m_counter_to_action[counterid]).cost // get_adjusted_cost(strips_task.get_action(m_counter_to_action[counterid]).cost)
                              : 0;
        if (counter.preconditions == 0) {
            m_true_conjunction.pre_of.push_back(&m_counters[counterid]);
            counter.preconditions = 1;
        }
        m_counter_precondition.push_back(temp_precondition[counterid]);
    }

    for (unsigned p = 0; p < strips::num_facts(); p++) {
        ConjunctionData &conj = m_conjunction_data[p];
        m_conjunction_achievers.push_back(temp_achievers[p]);
        for (const unsigned &counterid : temp_achievers[p]) {
            m_counters[counterid].effect = &conj;
        }
        conj.pre_of.reserve(m_counters_with_fact_precondition.size());
        for (const unsigned &counterid : m_counters_with_fact_precondition[p]) {
            conj.pre_of.push_back(&m_counters[counterid]);
        }
    }

    // printf("Generating conjunctions out of %zu unit conjunctions and %zu counters.\n",
    //        strips::num_facts(),
    //        m_counters.size());

    m_num_atomic_counters = m_counters.size();

    if (m > 1) {
        struct EnumData {
            int var;
            int val;
            int succ_var;
            EnumData(int var, int dom)
                : var(var), val(dom), succ_var(var)
            {}
        };
        std::vector<EnumData> queue;
        std::vector<unsigned> conj;
        for (int var = 0; var < task->get_num_variables(); var++) {
            queue.emplace_back(var, task->get_variable_domain_size(var));
            conj.push_back(-1);
            while (!queue.empty()) {
                EnumData& e = queue.back();
                conj.pop_back();
                if (--e.val < 0) {
                    queue.pop_back();
                    continue;
                }
                conj.push_back(strips::get_fact_id(e.var, e.val));
                insert_conjunction_and_update_data_structures(conj);
                if (conj.size() < m && ++e.succ_var < task->get_num_variables()) {
                    queue.emplace_back(e.succ_var, task->get_variable_domain_size(e.succ_var));
                    conj.push_back(-1);
                }
            }
            assert(conj.empty());
        }
    }

    // std::cout << "Conjunction set initialization method: ";
    // if (m_conjunction_set_initializer != NULL) {
    //     m_conjunction_set_initializer->print_options();
    //     m_conjunction_set_initializer->compute_conjunctions_set(this);
    // } else {
    //     std::cout << "all unit conjunctions" << std::endl;
    // }

    if (m_nogood_formula != nullptr) {
        m_nogood_formula->initialize();
    }
}

void HCHeuristic::get_satisfied_conjunctions(
    const std::vector<unsigned> &conj,
    std::vector<unsigned> &ids)
{
    std::fill(m_subset_count.begin(), m_subset_count.end(), 0);
    for (const unsigned &p : conj) {
        for (const unsigned &c : m_fact_to_conjunctions[p]) {
            if (++m_subset_count[c] == m_conjunction_size[c]) {
                ids.emplace_back(c);
            }
        }
    }
    std::sort(ids.begin(), ids.end());
}

void HCHeuristic::get_satisfied_conjunctions(
    const State &fdr_state,
    std::vector<unsigned> &hc_state)
{
    std::fill(m_subset_count.begin(), m_subset_count.end(), 0);
    for (int var = 0; var < task->get_num_variables(); var++) {
        unsigned p = strips::get_fact_id(var, fdr_state[var]);
        for (const unsigned &pq : m_fact_to_conjunctions[p]) {
            if (++m_subset_count[pq] == m_conjunction_size[pq]) {
                hc_state.push_back(pq);
            }
        }
    }
    std::sort(hc_state.begin(), hc_state.end());
}

unsigned HCHeuristic::lookup_conjunction_id(
    const std::vector<unsigned> &conj,
    std::vector<unsigned> &conj_subsets,
    std::vector<unsigned> &conj_supersets)
{
    std::fill(m_subset_count.begin(), m_subset_count.end(), 0);
    for (const unsigned &p : conj) {
        for (const unsigned &c : m_fact_to_conjunctions[p]) {
            if (++m_subset_count[c] == m_conjunction_size[c]) {
                if (m_conjunction_size[c] == conj.size()) {
                    return c;
                }
                conj_subsets.push_back(c);
            } else if (m_subset_count[c] == conj.size()) {
                conj_supersets.push_back(c);
            }
        }
    }
    std::sort(conj_subsets.begin(), conj_subsets.end());
    std::sort(conj_supersets.begin(), conj_supersets.end());
    return m_conjunction_data.size();
}

void HCHeuristic::update_fact_conjunction_mapping(
    const std::vector<unsigned> &conj,
    unsigned conjid)
{
    m_conjunction_size.push_back(conj.size());
    m_subset_count.push_back(0);
    for (const unsigned &p : conj) {
        m_fact_to_conjunctions[p].push_back(conjid);
    }
}

std::pair<unsigned, bool>
HCHeuristic::insert_conjunction_and_update_data_structures(
    const std::vector<unsigned> &conj,
    int cost)
{
#ifndef NDEBUG
    assert(std::is_sorted(conj.begin(), conj.end()));
    std::vector<unsigned> __debug_tbv(conj);
    assert(std::unique(__debug_tbv.begin(),
                       __debug_tbv.end()) == __debug_tbv.end());
#endif

    std::vector<unsigned> conj_subsets;
    std::vector<unsigned> conj_supersets;
    unsigned newconjid = lookup_conjunction_id(conj,
                         conj_subsets,
                         conj_supersets);
    if (newconjid < m_conjunction_data.size()) {
        ConjunctionData *data = &m_conjunction_data[newconjid];
        if (data->cost != ConjunctionData::UNACHIEVED
            && (cost == ConjunctionData::UNACHIEVED || cost > data->cost)) {
            for (Counter *counter : data->pre_of) {
                if (cost == ConjunctionData::UNACHIEVED) {
                    counter->unsat++;
                }
            }
            data->cost = cost;
        }
        return std::pair<unsigned, bool>(newconjid, false);
    }

#ifndef NDEBUG
    for (unsigned cid : conj_subsets) {
        assert(std::includes(conj.begin(), conj.end(),
                             m_conjunctions[cid].begin(), m_conjunctions[cid].end()));
    }
    for (unsigned cid : conj_supersets) {
        assert(std::includes(m_conjunctions[cid].begin(), m_conjunctions[cid].end(),
                             conj.begin(), conj.end()));
    }
#endif

    for (const unsigned &sup : conj_supersets) {
        ConjunctionData& data = m_conjunction_data[sup];
        if (cost == ConjunctionData::UNACHIEVED) {
            data.cost = ConjunctionData::UNACHIEVED;
        } else if (cost > data.cost) {
            data.cost = cost;
        }
    }

    m_conjunctions.push_back(conj);
    m_conjunction_data.push_back(ConjunctionData(newconjid));
    m_conjunction_achievers.push_back(std::vector<unsigned>());
    ConjunctionData &new_conjdata = m_conjunction_data[newconjid];
    new_conjdata.cost = cost;

    assert(m_conjunction_data.size() == m_conjunctions.size());
    assert(m_conjunction_achievers.size() == m_conjunctions.size());

    // updating existing counters (extending preconditions)
    std::vector<unsigned> in_counter_pre(m_counters.size(), 0);
    for (const unsigned &p : conj) {
        for (const unsigned &c : m_counters_with_fact_precondition[p]) {
            if (++in_counter_pre[c] == conj.size()) {
                if (!c_prune_subsumed_preconditions
                    || !set_utils::intersects(m_counter_precondition[c], conj_supersets)) {
                    Counter *counter = &m_counters[c];
                    // assert(counter->unsat != 0 || counter->max_pre != NULL);
                    if (c_prune_subsumed_preconditions) {
                        set_utils::inplace_difference(
                            m_counter_precondition[c],
                            conj_subsets,
                        [this, counter](const unsigned & d) {
                            ConjunctionData &data = m_conjunction_data[d];
                            assert(std::find(data.pre_of.begin(),
                                             data.pre_of.end(),
                                             counter) != data.pre_of.end());
                            data.pre_of.erase(std::find(data.pre_of.begin(),
                                                        data.pre_of.end(),
                                                        counter));
                            counter->preconditions--;
                            if (!data.achieved()) {
                                counter->unsat--;
                            }
                        });
                    }
                    m_counter_precondition[c].push_back(newconjid);
                    new_conjdata.pre_of.push_back(counter);
                    counter->preconditions++;
                    if (cost == ConjunctionData::UNACHIEVED) {
                        counter->unsat++;
                    } else if (counter->unsat == 0 &&
                               (counter->max_pre == NULL || counter->max_pre->cost < cost)) {
                        counter->max_pre = &new_conjdata;
                    }
                }
            }
        }
    }

    const strips::Task &strips_task = strips::get_task();
    std::vector<int> achieving_actions(strips_task.num_actions(), 0);
    strips_task.compute_achievers(achieving_actions, conj);
    std::vector<unsigned> regression;
    std::vector<std::pair<unsigned, unsigned> > precondition;
    std::vector<bool> precondition_pruned;
    for (unsigned op = 0; op < strips_task.num_actions(); op++) {
        if (achieving_actions[op] == 1) {
            const strips::Action &action = strips_task.get_action(op);

            // compute precondition of new counter: do regression
            std::set_union(conj.begin(), conj.end(),
                           action.pre.begin(), action.pre.end(),
                           std::back_inserter(regression));
            set_utils::inplace_difference(regression, action.add);
            assert(!strips_task.contains_mutex(regression));

            // find corresponding conjunctions (satisfied in this regression)
            get_satisfied_conjunctions(regression,
            [this, &precondition](const unsigned & cid) {
                precondition.emplace_back(m_conjunction_size[cid], cid);
            });
            precondition_pruned.resize(precondition.size(), false);

            // find those conjunctions that are subsumed by others
            if (c_prune_subsumed_preconditions) {
                std::sort(precondition.begin(), precondition.end());
                for (unsigned i = precondition.size() - 1; i < precondition.size(); i--) {
                    if (precondition_pruned[i]) {
                        continue;
                    }
                    const std::vector<unsigned> &conj_i =
                        m_conjunctions[precondition[i].second];
                    for (unsigned j = 0; j < i
                         && precondition[j].first < precondition[i].first; j++) {
                        if (precondition_pruned[j]) {
                            continue;
                        }
                        const std::vector<unsigned> &conj_j =
                            m_conjunctions[precondition[j].second];
                        if (std::includes(conj_i.begin(), conj_i.end(), conj_j.begin(), conj_j.end())) {
                            precondition_pruned[j] = true;
                        }
                    }
                }
            }

            // create new counter
            unsigned counterid = m_counters.size();
            m_counters.push_back(Counter(counterid,
                                         action.cost, //get_adjusted_cost(action.cost),
                                         0,
                                         &new_conjdata));
            m_counter_precondition.push_back(std::vector<unsigned>());
            m_counter_to_action.push_back(op);
            m_conjunction_achievers[newconjid].push_back(counterid);
            assert(m_counters.size() == m_counter_precondition.size());
            assert(m_counters.size() == m_counter_to_action.size());
            std::vector<unsigned> &counter_precondition = m_counter_precondition[counterid];
            Counter *counter = &m_counters[counterid];
            counter->unsat = 0;
            for (unsigned i = 0; i < precondition.size(); i++) {
                if (!precondition_pruned[i]) {
                    counter_precondition.push_back(precondition[i].second);
                    counter->preconditions++;
                    ConjunctionData &pre = m_conjunction_data[precondition[i].second];
                    pre.pre_of.push_back(counter);
                    counter->unsat += !pre.achieved();
                    if (counter->unsat == 0 &&
                        (counter->max_pre == NULL || counter->max_pre->cost < pre.cost)) {
                        counter->max_pre = &pre;
                    }
                }
            }
            if (counter->preconditions == 0) {
                counter->preconditions = 1;
                counter->max_pre = &m_true_conjunction;
                m_true_conjunction.pre_of.push_back(counter);
            }
            std::sort(counter_precondition.begin(), counter_precondition.end());

            for (const unsigned &p : regression) {
                m_counters_with_fact_precondition[p].push_back(counterid);
            }

            regression.clear();
            precondition.clear();
            precondition_pruned.clear();

            assert(counter->unsat != 0 || counter->max_pre != NULL);
        }
#ifndef NDEBUG
        else {
            const strips::Action &action = strips_task.get_action(op);
            if (set_utils::intersects(conj, action.add)
                && !set_utils::intersects(conj, action.del)) {
                std::set_union(conj.begin(), conj.end(),
                               action.pre.begin(), action.pre.end(),
                               std::back_inserter(regression));
                set_utils::inplace_difference(regression, action.add);
                assert(strips_task.contains_mutex(regression));
                regression.clear();
            }
        }
#endif
    }

    update_fact_conjunction_mapping(conj, newconjid);
    if (m_nogood_formula != nullptr) {
        m_nogood_formula->notify_on_new_conjunction(newconjid);
    }

    return std::pair<unsigned, bool>(newconjid, true);
}

unsigned HCHeuristic::insert_conjunction(const std::vector<unsigned> &conj,
        int cost)
{
    assert(m_conjunction_data.size() == m_conjunctions.size());
    unsigned res = m_conjunctions.size();
    m_conjunctions.push_back(conj);
    m_conjunction_data.push_back(ConjunctionData(res, cost));
    m_conjunction_achievers.push_back(std::vector<unsigned>());
    update_fact_conjunction_mapping(conj, res);
    return res;
}

unsigned HCHeuristic::create_counter(unsigned action_id,
                                     int action_cost,
                                     ConjunctionData *eff)
{
    unsigned res = m_counters.size();
    assert(m_counter_precondition.size() == res
           && m_counter_to_action.size() == res);
    m_counters.push_back(Counter(res, action_cost, eff)); // get_adjusted_cost(action_cost)
    m_counter_precondition.push_back(std::vector<unsigned>());
    m_counter_to_action.push_back(action_id);
    m_conjunction_achievers[eff->id].push_back(res);
    return res;
}

int
HCHeuristic::compute_heuristic_for_facts(const std::vector<unsigned>& fact_ids)
{
    m_state.clear();
    get_satisfied_conjunctions(fact_ids, m_state);
    if (c_nogood_evaluation_enabled
        && m_nogood_formula != nullptr) {
        int h = m_nogood_formula->evaluate_formula_quantitative(fact_ids);
        if (h == DEAD_END) {
#ifndef NDEBUG
            cleanup_previous_computation();
            assert(compute_heuristic(m_state) == DEAD_END);
#endif
            return DEAD_END;
        }
        if (cost_bound_ >= 0 && cost_bound_ - g_value_ < h) {
#ifndef NDEBUG
            cleanup_previous_computation();
            assert(compute_heuristic(m_state) >= h);
#endif
            return h;
        }
    }
    m_hc_evaluations++;
    cleanup_previous_computation();
    return compute_heuristic(m_state);
}

int
HCHeuristic::evaluate_partial_state(const PartialState& state)
{
    assert(std::is_sorted(state.begin(), state.end()));
    std::vector<unsigned> fact_ids;
    unsigned i = 0;
    for (int var = 0; var < task->get_num_variables(); var++) {
        if (i < state.size() && state[i].first == var) {
            fact_ids.push_back(strips::get_fact_id(state[i]));
            i++;
        } else {
            for (int val = 0; val < task->get_variable_domain_size(var); val++) {
                fact_ids.push_back(strips::get_fact_id(var, val));
            }
        }
    }
    // TODO nogood learning currently no implemented for partial states
    // if (res == DEAD_END && c_nogood_evaluation_enabled) {
    //     m_nogood_formula->refine_formula(state);
    // }
    return compute_heuristic_for_facts(fact_ids);
}

int HCHeuristic::compute_heuristic(const State &state)
{
    std::vector<unsigned> fact_ids;
    for (int var = 0; var < task->get_num_variables(); var++) {
        fact_ids.push_back(strips::get_fact_id(var, state[var]));
    }
    int res = compute_heuristic_for_facts(fact_ids);
    if (c_nogood_evaluation_enabled) {
        if (res == DEAD_END || (cost_bound_ >= 0 && cost_bound_ - g_value_ < res)) {
            m_nogood_formula->refine_formula(state, res == DEAD_END ? -1 : (cost_bound_ - g_value_));
        }
    }
    return res;
}

int HCHeuristic::compute_heuristic_incremental(
    const std::vector<unsigned> &new_facts,
    std::vector<unsigned> &reachable)
{
    for (const unsigned &p : new_facts) {
        for (const unsigned &cid : m_fact_to_conjunctions[p]) {
            if (++m_subset_count[cid] == m_conjunction_size[cid]) {
                const ConjunctionData &data = m_conjunction_data[cid];
                if (!data.achieved()) {
                    reachable.push_back(cid);
                }
            }
        }
    }
    return compute_heuristic_get_reachable_conjunctions(reachable);
}

void HCHeuristic::revert_incremental_computation(
    const std::vector<unsigned> &new_facts,
    const std::vector<unsigned> &reachable_conjunctions)
{
    for (const unsigned &p : new_facts) {
        for (const unsigned &cid : m_fact_to_conjunctions[p]) {
            --m_subset_count[cid];
        }
    }
    for (const unsigned &cid : reachable_conjunctions) {
        ConjunctionData &data = m_conjunction_data[cid];
        data.cost = ConjunctionData::UNACHIEVED;
        for (Counter *counter : data.pre_of) {
            counter->unsat++;
            counter->max_pre = NULL;
        }
    }
    assert(m_goal_conjunction.cost = ConjunctionData::UNACHIEVED);
}

bool HCHeuristic::set_early_termination(bool t)
{
    bool x = c_early_termination;
    c_early_termination = t;
    return x;
}

bool HCHeuristic::set_early_termination_and_nogoods(bool e)
{
    bool x = c_early_termination;
    c_early_termination = e;
    c_nogood_evaluation_enabled = e && m_nogood_formula != nullptr;
    return x;
}

double HCHeuristic::get_counter_ratio() const
{
    return ((double) m_counters.size()) / m_num_atomic_counters;
}

bool HCHeuristic::prune_subsumed_preconditions() const
{
    return c_prune_subsumed_preconditions;
}

size_t HCHeuristic::num_conjunctions() const
{
    return m_conjunction_data.size();
}

size_t HCHeuristic::num_counters() const
{
    return m_counters.size();
}

size_t HCHeuristic::num_atomic_counters() const
{
    return m_num_atomic_counters;
}

unsigned HCHeuristic::get_action_id(unsigned counter) const
{
    return m_counter_to_action[counter];
}

unsigned HCHeuristic::get_conjunction_size(unsigned cid) const
{
    return m_conjunction_size[cid];
}

const std::vector<unsigned> &HCHeuristic::get_conjunction_achievers(
    unsigned cid) const
{
    return m_conjunction_achievers[cid];
}

const std::vector<unsigned> &HCHeuristic::get_conjunction(unsigned cid) const
{
    return m_conjunctions[cid];
}

ConjunctionData &HCHeuristic::get_conjunction_data(unsigned id)
{
    return m_conjunction_data[id];
}

Counter &HCHeuristic::get_counter(unsigned id)
{
    return m_counters[id];
}

ConjunctionData &HCHeuristic::get_true_conjunction_data()
{
    return m_true_conjunction;
}

ConjunctionData &HCHeuristic::get_goal_conjunction_data()
{
    return m_goal_conjunction;
}

Counter &HCHeuristic::get_goal_counter()
{
    return m_counters[m_goal_counter];
}

std::vector<unsigned> &HCHeuristic::get_counters_with_fact_precondition(
    unsigned p)
{
    return m_counters_with_fact_precondition[p];
}

std::vector<unsigned> &HCHeuristic::get_counter_precondition(unsigned id)
{
    return m_counter_precondition[id];
}

int HCHeuristic::get_cost_of_subgoal(const std::vector<unsigned> &subgoal)
{
    assert(!c_early_termination);
    int cost = 0;
    get_satisfied_conjunctions(subgoal, [this, &cost](const unsigned & cid) {
        const ConjunctionData &data = get_conjunction_data(cid);
        if (!data.achieved()) {
            cost = ConjunctionData::UNACHIEVED;
        } else if (cost != ConjunctionData::UNACHIEVED && cost < data.cost) {
            cost = data.cost;
        }
    });
    return cost == ConjunctionData::UNACHIEVED
           ? std::numeric_limits<int>::max()
           : cost;
}

bool HCHeuristic::is_subgoal_reachable(const std::vector<unsigned> &subgoal)
{
    return get_cost_of_subgoal(subgoal) < std::numeric_limits<int>::max();
}

void HCHeuristic::mark_unachieved(unsigned cid)
{
    forall_superset_conjunctions(get_conjunction(cid),
    [this](const unsigned & id) {
        ConjunctionData &data = get_conjunction_data(id);
        if (data.achieved()) {
            data.cost = ConjunctionData::UNACHIEVED;
            for (Counter *c : data.pre_of) {
                c->unsat++;
            }
        }
    });
}

void HCHeuristic::print_statistics() const
{
    printf("hC over %zu conjunctions and %zu (%.6f) counters.\n",
           m_conjunction_data.size(),
           m_counters.size(),
           (((double) m_counters.size()) / m_num_atomic_counters));
    // printf("Total time spent on hC evaluation: %.6fs\n",
    //        get_evaluation_time());
    if (m_nogood_formula != nullptr) {
        m_nogood_formula->print_statistics();
    }
    if (store_conjunctions_ != "") {
        std::ofstream out;
        out.open(store_conjunctions_);
        for (unsigned i = 0; i < m_conjunctions.size(); i++) {
            if (m_conjunctions[i].size() > 1) {
                bool isfirst = true;
                for (const auto& fid : m_conjunctions[i]) {
                    const auto f = strips::get_variable_assignment(fid);
                    out << (isfirst ? "" : " | ") <<
                        task->get_fact_name(FactPair(f.first, f.second));
                    isfirst = false;
                }
                out << std::endl;
            }
        }
        out.close();
    }
}

void HCHeuristic::print_options() const
{
}

bool HCHeuristic::supports_partial_state_evaluation() const
{
    return true;
}

int HCHeuristic::evaluate(const State& state, int g)
{
    g_value_ = g;
    return compute_heuristic(state);
}

EvaluationResult 
HCHeuristic::compute_result(
    EvaluationContext &eval_context)
{
    EvaluationResult result;
    const State &state = eval_context.get_state();
    g_value_ = eval_context.get_g_value();
    int heuristic = compute_heuristic(state);
    result.set_count_evaluation(true);
    if (heuristic == DEAD_END) {
        heuristic = EvaluationResult::INFTY;
    }
    result.set_evaluator_value(heuristic);
    return result;
}

void HCHeuristic::set_abstract_task(std::shared_ptr<AbstractTask> task)
{
    Heuristic::set_abstract_task(task);

    //std::cout << "before: ";
    //for (auto i : strips::get_task().get_goal()) {
    //    std::cout << " " << i;
    //}
    //std::cout << std::endl;
    for (auto g : strips::get_task().get_goal()) {
        auto& refs = m_counters_with_fact_precondition[g];
        auto it = std::find(refs.begin(), refs.end(), m_goal_counter);
        assert(it != refs.end());
        refs.erase(it);
    }
    std::vector<unsigned> goal_conjunctions;
    get_satisfied_conjunctions(strips::get_task().get_goal(), goal_conjunctions);
    for (const unsigned& id : goal_conjunctions) {
        std::vector<Counter*>& counters = m_conjunction_data[id].pre_of;
        auto it = std::find(counters.begin(), counters.end(), &m_counters[m_goal_counter]);
        if (it != counters.end()) {
            counters.erase(it);
        }
    }
    goal_conjunctions.clear();

    strips::update_goal_set(*task);
    for (auto g : strips::get_task().get_goal()) {
        m_counters_with_fact_precondition[g].push_back(m_goal_counter);
    }
    //std::cout << "after: ";
    //for (auto i : strips::get_task().get_goal()) {
    //    std::cout << " " << i;
    //}
    //std::cout << std::endl;
    get_satisfied_conjunctions(strips::get_task().get_goal(), goal_conjunctions);
    std::vector<bool> pruned(goal_conjunctions.size(), false);
    if (c_prune_subsumed_preconditions) {
        std::vector<std::pair<unsigned, unsigned> > gs; gs.reserve(goal_conjunctions.size());
        for (unsigned i : goal_conjunctions) {
            gs.emplace_back(m_conjunctions[i].size(), i);
        }
        std::sort(gs.begin(), gs.end());
        for (unsigned i = gs.size() - 1; i < gs.size(); i--) {
            goal_conjunctions[i] = gs[i].second;
            const std::vector<unsigned> &conj_i = m_conjunctions[gs[i].second];
            for (unsigned j = 0; j < i
                    && gs[j].first < gs[i].first; j++) {
                if (pruned[j]) {
                    continue;
                }
                const std::vector<unsigned> &conj_j =
                    m_conjunctions[gs[j].second];
                if (std::includes(conj_i.begin(), conj_i.end(), conj_j.begin(), conj_j.end())) {
                    pruned[j] = true;
                }
            }
        }
    }
    for (const unsigned& id : goal_conjunctions) {
        std::vector<Counter*>& counters = m_conjunction_data[id].pre_of;
        counters.push_back(&m_counters[m_goal_counter]);
    }
    m_counters[m_goal_counter].preconditions = goal_conjunctions.size();

    if (m_nogood_formula != nullptr) {
        m_nogood_formula->synchronize_goal(task);
    }
}

void
HCHeuristic::dump_conjunction(const std::vector<unsigned>& conjunction) const
{
    dump_conjunction(std::cout, conjunction);
}

void
HCHeuristic::dump_conjunction(std::ostream& out,
                              const std::vector<unsigned>& conjunction) const
{
    bool sep = false;
    out << "[";
    for (const auto& fact_id : conjunction) {
        auto fact = strips::get_variable_assignment(fact_id);
        out << (sep ? ", " : "")
            << task->get_fact_name(FactPair(fact.first, fact.second));
        sep = true;
    }
    out << "]";
}

void HCHeuristic::dump_conjunctions(std::ostream& out) const
{
    for (unsigned i = 0; i < m_conjunctions.size(); i++) {
        dump_conjunction(out, m_conjunctions[i]);
        out << std::endl;
    }
}

void HCHeuristic::add_options_to_parser(options::OptionParser &parser)
{
    Heuristic::add_options_to_parser(parser);
    parser.add_option<bool>("prune_subsumed_preconditions", "", "false");
    parser.add_option<bool>("nogoods", "", "true");
    parser.add_option<int>("m", "", "0");
    parser.add_option<int>("cost_bound", "", "-1");
    parser.add_option<std::string>("conjs_in", "", options::OptionParser::NONE);
    parser.add_option<std::string>("conjs_out", "", options::OptionParser::NONE);
    // parser.add_option<NoGoodFormula *>("nogoods", "", options::OptionParser::NONE);
}

bool HCHeuristicUnitCost::enqueue_if_necessary(
    ConjunctionData *eff,
    const int &lvl)
{
    if (!eff->achieved()) {
        eff->cost = lvl;
        m_open.push_back(eff);
        return true;
    }
    return false;
}

void HCHeuristicUnitCost::cleanup_previous_computation()
{
    // clear data structures
    m_true_conjunction.cost = ConjunctionData::UNACHIEVED;
    m_goal_conjunction.cost = ConjunctionData::UNACHIEVED;
    for (unsigned x = 0; x < m_conjunction_data.size(); x++) {
        m_conjunction_data[x].cost = ConjunctionData::UNACHIEVED;
    }
    for (unsigned x = 0; x < m_counters.size(); x++) {
        m_counters[x].unsat = m_counters[x].preconditions;
        m_counters[x].max_pre = NULL;
    }
    m_open.clear();
}

int HCHeuristicUnitCost::compute_heuristic(const std::vector<unsigned> &state)
{
    enqueue_if_necessary(&m_true_conjunction, 0);
    for (const unsigned &p : state) {
        enqueue_if_necessary(&m_conjunction_data[p], 0);
    }
    int level = 0;
    unsigned next_level = m_open.size();
    for (unsigned i = 0; i < m_open.size()
         && (!c_early_termination || !m_goal_conjunction.achieved());
         i++) {
        if (i == next_level) {
            next_level = m_open.size();
            level++;
        }
        ConjunctionData *eff = m_open[i];
        for (Counter *c : eff->pre_of) {
            if (--(c->unsat) == 0) {
                c->max_pre = eff;
                enqueue_if_necessary(c->effect, level + 1);
            }
        }
    }
    assert(!m_goal_conjunction.achieved() || m_goal_conjunction.cost > 0);
    return m_goal_conjunction.achieved() ? m_goal_conjunction.cost - 1 : DEAD_END;
}

int HCHeuristicUnitCost::compute_heuristic_get_reachable_conjunctions(
    std::vector<unsigned> &reachable)
{
    unsigned i = m_open.size();
    enqueue_if_necessary(&m_true_conjunction, 0);
    for (const unsigned &p : reachable) {
        enqueue_if_necessary(&m_conjunction_data[p], 0);
    }
    int level = 0;
    unsigned next_level = m_open.size();
    for (; i < m_open.size()
         && (!c_early_termination || !m_goal_conjunction.achieved());
         i++) {
        if (i == next_level) {
            next_level = m_open.size();
            level++;
        }
        ConjunctionData *eff = m_open[i];
        for (Counter *c : eff->pre_of) {
            if (--(c->unsat) == 0) {
                c->max_pre = eff;
                if (enqueue_if_necessary(c->effect, level + 1)
                    && c->effect != &m_goal_conjunction) {
                    reachable.push_back(c->effect->id);
                }
            }
        }
    }
    return m_goal_conjunction.achieved() ? m_goal_conjunction.cost - 1 : DEAD_END;
}

bool HCHeuristicGeneralCost::enqueue_if_necessary(
    ConjunctionData *eff,
    const int &cost)
{
    bool res = !eff->achieved();
    if (!eff->achieved() || eff->cost > cost) {
        eff->cost = cost;
        m_open.push(cost, eff);
    }
    return res;
}

bool HCHeuristicGeneralCost::enqueue_if_necessary(
    ConjunctionData *eff)
{
    bool res = !eff->achieved();
    if (!eff->achieved()) {
        eff->cost = 0;
        m_open.push(0, eff);
    }
    return res;
}

void HCHeuristicGeneralCost::cleanup_previous_computation()
{
    // clear data structures
    m_true_conjunction.cost = ConjunctionData::UNACHIEVED;
    m_goal_conjunction.cost = ConjunctionData::UNACHIEVED;
    for (unsigned x = 0; x < m_conjunction_data.size(); x++) {
        m_conjunction_data[x].cost = ConjunctionData::UNACHIEVED;
    }
    for (unsigned x = 0; x < m_counters.size(); x++) {
        m_counters[x].unsat = m_counters[x].preconditions;
        m_counters[x].max_pre = NULL;
    }
    m_open.clear();
}

int HCHeuristicGeneralCost::compute_heuristic(
    const std::vector<unsigned> &state)
{
    enqueue_if_necessary(&m_true_conjunction, 0);
    for (const unsigned &p : state) {
        enqueue_if_necessary(&m_conjunction_data[p], 0);
    }
    while (!m_open.empty()) {
        std::pair<int, ConjunctionData *> elem = m_open.pop();
        if (elem.second->cost < elem.first) {
            continue;
        }
        if (c_early_termination && elem.second == &m_goal_conjunction) {
            break;
        }
        for (Counter *counter : elem.second->pre_of) {
            if (--(counter->unsat) == 0) {
                counter->max_pre = elem.second;
                enqueue_if_necessary(counter->effect,
                                     elem.first + counter->action_cost);
            }
        }
    }
    return m_goal_conjunction.achieved() ? m_goal_conjunction.cost : DEAD_END;
}

int HCHeuristicGeneralCost::compute_heuristic_get_reachable_conjunctions(
    std::vector<unsigned> &reachable_conjunctions)
{
    assert(m_open.empty());
    enqueue_if_necessary(&m_true_conjunction);
    for (const unsigned &p : reachable_conjunctions) {
        enqueue_if_necessary(&m_conjunction_data[p]);
    }
    while (!m_open.empty()) {
        std::pair<int, ConjunctionData *> elem = m_open.pop();
        if (c_early_termination && elem.second == &m_goal_conjunction) {
            break;
        }
        for (Counter *counter : elem.second->pre_of) {
            if (--(counter->unsat) == 0) {
                counter->max_pre = elem.second;
                if (enqueue_if_necessary(counter->effect)
                    && counter->effect != &m_goal_conjunction) {
                    reachable_conjunctions.push_back(counter->effect->id);
                }
            }
        }
    }
    return m_goal_conjunction.achieved() ? m_goal_conjunction.cost : DEAD_END;
}

int UCHeuristic::compute_heuristic(const std::vector<unsigned> &state)
{
    return HCHeuristicUnitCost::compute_heuristic(state) == DEAD_END ? DEAD_END : 0;
}

int UCHeuristic::compute_heuristic_get_reachable_conjunctions(
    std::vector<unsigned> &reachable)
{
    return HCHeuristicUnitCost::compute_heuristic(reachable) == DEAD_END ? DEAD_END : 0;
}


}
}

static std::shared_ptr<Heuristic>
_parse(options::OptionParser& parser)
{
    conflict_driven_learning::hc_heuristic::HCHeuristic::add_options_to_parser(parser);
    options::Options opts = parser.parse();
    if (!parser.dry_run()) {
        TaskProxy proxy(*opts.get<std::shared_ptr<AbstractTask>>("transform"));
        if (task_properties::is_unit_cost(proxy)) {
            return std::make_shared< conflict_driven_learning::hc_heuristic::HCHeuristicUnitCost>(opts);
        } else {
            return std::make_shared< conflict_driven_learning::hc_heuristic::HCHeuristicGeneralCost>(opts);
        }
    }
    return NULL;
}

static std::shared_ptr<Heuristic>
_parse_uc(options::OptionParser& parser)
{
    conflict_driven_learning::hc_heuristic::HCHeuristic::add_options_to_parser(parser);
    options::Options opts = parser.parse();
    if (!parser.dry_run()) {
        return std::make_shared< conflict_driven_learning::hc_heuristic::UCHeuristic>(opts);
    }
    return NULL;
}

static Plugin<Evaluator> _plugin_hc("hc", _parse);
static Plugin<Evaluator> _plugin_uc("uc", _parse_uc);
