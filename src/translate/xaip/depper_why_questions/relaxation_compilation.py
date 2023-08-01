from .relaxed_task import RelaxedTask
from sas_tasks import *


def find_var_value_id(sas_variables, fact):
    negated = fact.startswith('!')
    pure_fact = fact.replace('! ', '')
    for var_id, variable in enumerate(sas_variables):
        for value_id, value in enumerate(variable):
            if (negated and 'NegatedAtom ' + pure_fact == value) or (not negated and 'Atom ' + pure_fact == value):
                return var_id, value_id
    assert False, fact + ' not in ' + str(sas_variables)


class RelaxationCompilation:

    def __init__(self, relaxed_tasks, sas_task):
        self.relaxed_tasks = relaxed_tasks
        self.init_relax_phase_var_id = None
        self.relaxed_task_indicator_var_id = None
        self.sas_task = sas_task

    def compile_tasks_part_one(self):
        # flag variable for indication of init and planning phase
        self.init_relax_phase_var_id = len(self.sas_task.variables.value_names)
        init_relax_phase_values = ['init_relax_phase', 'planning_phase']

        self.sas_task.variables.value_names.append(init_relax_phase_values)
        self.sas_task.variables.ranges.append(len(init_relax_phase_values))
        self.sas_task.variables.axiom_layers.append(-1)

        # start with init relax phase
        self.sas_task.init.values.append(0)

        # relaxed task indicator flag
        self.relaxed_task_indicator_var_id = len(self.sas_task.variables.value_names)
        relaxed_task_indicator_values = ['original_task'] + ['relaxed_task(' + t.name + ')' for t in self.relaxed_tasks]

        self.sas_task.variables.value_names.append(relaxed_task_indicator_values)
        self.sas_task.variables.ranges.append(len(relaxed_task_indicator_values))
        self.sas_task.variables.axiom_layers.append(-1)

        # start with original task
        self.sas_task.init.values.append(0)

    def compile_tasks_part_two(self):
        for op in self.sas_task.operators:
            # original operators can only be executed in the planning phase
            op.pre_post.append((self.init_relax_phase_var_id, 1, 1, []))

        # action which change the initial values and to the planning phase
        for i, relaxed_task in enumerate(self.relaxed_tasks):
            name = '(change_to_' + relaxed_task.name + ')'
            pre_post = [(self.init_relax_phase_var_id, 0, 1, []), (self.relaxed_task_indicator_var_id, 0, i + 1, [])]
            for var, value in relaxed_task.init:
                pre_post.append((var, self.sas_task.init.values[var], value, []))
            self.sas_task.operators.append(SASOperator(name, [], pre_post, 0))
