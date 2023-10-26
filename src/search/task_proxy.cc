#include "task_proxy.h"

#include "axioms.h"
#include "state_registry.h"

#include "task_utils/causal_graph.h"
#include "task_utils/task_properties.h"

#include <iostream>

using namespace std;

const int PartialAssignment::UNASSIGNED = -1;

PartialAssignment::PartialAssignment(const AbstractTask &task)  : task(&task), values(nullptr) {}
PartialAssignment::PartialAssignment(const AbstractTask &task, vector<int> &&values)
    : task(&task), values(make_shared<vector<int>>(std::move(values))) {
    assert(static_cast<int>(this->values->size()) == task.get_num_variables());
}

PartialAssignment::PartialAssignment()  : task(nullptr), values(nullptr) {}

PartialAssignment::PartialAssignment(const PartialAssignment &assignment, vector<int> &&values)
    : task(assignment.task), values(make_shared<vector<int>>(std::move(values))) {
    assert(static_cast<int>(this->values->size()) == task->get_num_variables());
}

State::State(const AbstractTask &task, const StateRegistry &registry,
             StateID id, const PackedStateBin *buffer)
    : PartialAssignment(task), registry(&registry), id(id), buffer(buffer),
      state_packer(&registry.get_state_packer()),
      num_variables(registry.get_num_variables()) {
    assert(id != StateID::no_state);
    assert(buffer);
    assert(num_variables == task.get_num_variables());
}

State::State(const AbstractTask &task, const StateRegistry &registry,
             StateID id, const PackedStateBin *buffer,
             vector<int> &&values)
    : State(task, registry, id, buffer) {
    assert(num_variables == static_cast<int>(values.size()));
    this->values = make_shared<vector<int>>(std::move(values));
}

State::State(const AbstractTask &task, vector<int> &&values)
    : PartialAssignment(task, std::move(values)), registry(nullptr), id(StateID::no_state), buffer(nullptr),
      state_packer(nullptr), num_variables(this->values->size()) {
    assert(num_variables == task.get_num_variables());
}

State::State()
    : PartialAssignment(), registry(nullptr), id(StateID::no_state), buffer(nullptr),
      state_packer(nullptr), num_variables(0) {
}

State State::get_unregistered_successor(const OperatorProxy &op) const {
    assert(!op.is_axiom());
    assert(task_properties::is_applicable(op, *this));
    assert(values);
    vector<int> new_values = get_unpacked_values();

    for (EffectProxy effect : op.get_effects()) {
        if (does_fire(effect, *this)) {
            FactPair effect_fact = effect.get_fact().get_pair();
            new_values[effect_fact.var] = effect_fact.value;
        }
    }

    if (task->get_num_axioms() > 0) {
        AxiomEvaluator &axiom_evaluator = g_axiom_evaluators[TaskProxy(*task)];
        axiom_evaluator.evaluate(new_values);
    }
    return State(*task, std::move(new_values));
}


const causal_graph::CausalGraph &TaskProxy::get_causal_graph() const {
    return causal_graph::get_causal_graph(task);
}



bool contains_mutex_with_variable(
    const AbstractTask *task, size_t var, const vector<int> &values,
    size_t first_var2_index = 0) {
    assert(utils::in_bounds(var, values));
    if (values[var] == PartialAssignment::UNASSIGNED) {
        return false;
    }
    FactPair fp(var, values[var]);

    for (size_t var2 = first_var2_index; var2 < values.size(); ++var2) {
        assert(utils::in_bounds(var2, values));
        if (var2 == var || values[var] == PartialAssignment::UNASSIGNED) {
            continue;
        }
        FactPair fp2(var2, values[var2]);
        if (task->are_facts_mutex(fp, fp2)) {
            return true;
        }
    }
    return false;
}

bool contains_mutex(const AbstractTask *task, const vector<int> &values) {
    for (size_t var = 0; var < values.size(); ++var) {
        if (contains_mutex_with_variable(task, var, values, var + 1)) {
            return true;
        }
    }
    return false;
}

/*
  Replace values[var] with non-mutex value. Return true iff such a
  non-mutex value could be found.
 */
static bool replace_with_non_mutex_value(
    const AbstractTask *task, vector<int> &values,
    const int idx_var, utils::RandomNumberGenerator &rng) {
    utils::in_bounds(idx_var, values);
    int old_value = values[idx_var];
    vector<int> domain(task->get_variable_domain_size(idx_var));
    iota(domain.begin(), domain.end(), 0);
    rng.shuffle(domain);
    for (int new_value : domain) {
        values[idx_var] = new_value;
        if (!contains_mutex_with_variable(task, idx_var, values)) {
            return true;
        }
    }
    values[idx_var] = old_value;
    return false;
}


static const int MAX_TRIES_EXTEND = 10000;
static bool replace_dont_cares_with_non_mutex_values(
    const AbstractTask *task, vector<int> &values,
    utils::RandomNumberGenerator &rng) {
    assert(values.size() == (size_t)task->get_num_variables());
    vector<int> vars_order(task->get_num_variables());
    iota(vars_order.begin(), vars_order.end(), 0);

    for (int round = 0; round < MAX_TRIES_EXTEND; ++round) {
        bool invalid = false;
        rng.shuffle(vars_order);
        vector<int> new_values = values;

        for (int idx_var : vars_order) {
            if (new_values[idx_var] == PartialAssignment::UNASSIGNED) {
                if (!replace_with_non_mutex_value(
                        task, new_values, idx_var, rng)) {
                    invalid = true;
                    break;
                }
            }
        }
        if (!invalid) {
            values = new_values;
            return true;
        }
    }
    return false;
}

bool PartialAssignment::violates_mutexes() const {
    return contains_mutex(task, get_unpacked_values());
}
pair<bool, State> PartialAssignment::get_full_state(
    bool check_mutexes,
    utils::RandomNumberGenerator &rng) const {
    vector<int> new_values = get_unpacked_values();
    bool success = true;
    if (check_mutexes) {
        if (contains_mutex(task, new_values)) {
            return make_pair(false, State(*task, std::move(new_values)));
        } else {
            success = replace_dont_cares_with_non_mutex_values(
                task, new_values, rng);
        }
    } else {
        for (VariableProxy var : VariablesProxy(*task)) {
            int &value = new_values[var.get_id()];
            if (value == PartialAssignment::UNASSIGNED) {
                int domain_size = var.get_domain_size();
                value = rng(domain_size);
            }
        }
    }
    return make_pair(success, State(*task, std::move(new_values)));
}


pair<bool, State> TaskProxy::convert_to_full_state(
    PartialAssignment &assignment,
    bool check_mutexes, utils::RandomNumberGenerator &rng) const {
    return assignment.get_full_state(check_mutexes, rng);
}
