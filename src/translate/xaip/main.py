import json

from . import AS_property
from . import action_sets as action_set_comp
from . import LTL_property
from . import G_property
from .parser import parse
from .general import ExplanationSetting
from .general.special_goals import Goal, set_goals
from .depper_why_questions import RelaxationCompilation


# TODO remove this class
class PredFact:

    def __init__(self, pred, args):
        self.pred = pred
        self.args = args

    @staticmethod
    def parse(s):
        parts = s.replace(')', '').split('(')
        return PredFact(parts[0], parts[1].replace(' ', '').split(','))

    def __repr__(self):
        return self.pred + '(' + ','.join(self.args) + ')'


class XPPFramework:

    def __init__(self, options, task):
        self.options = options
        self.task = task
        self.sas_task = None
        self.EXPSET = ExplanationSetting()

        self.parse()

    def parse(self):
        properties_path = self.options.explanation_settings
        if properties_path != "None" and properties_path != "PROPERTY":
            with open(properties_path, encoding='utf-8') as fh:
                json_encoding = json.load(fh)

            # build typeObjectMap
            typeObjectMap = {}
            for o in self.task.objects:
                if not o.type_name in typeObjectMap:
                    typeObjectMap[o.type_name] = []

                typeObjectMap[o.type_name].append(o.name)
            parse(json_encoding, typeObjectMap, self.EXPSET)

            print("# AS properties: " + str(len(self.EXPSET.get_action_set_properties())))
            print("# LTL properties: " + str(len(self.EXPSET.get_ltl_properties())))
            print("# G properties: " + str(len(self.EXPSET.get_goal_properties())))

    def run(self):
        print("----> run XPP framework")

        # relaxed tasks to sas_task
        print("#relaxed tasks: " +str(len(self.EXPSET.get_relaxed_tasks())))
        if not self.options.compile_relaxed_tasks:
            print("Relaxed tasks to SAS output")
        for relaxed_task in self.EXPSET.get_relaxed_tasks():
            relaxed_task.update_init_and_limits(self.sas_task)
            if not self.options.compile_relaxed_tasks:
                self.sas_task.addRelaxedTask(relaxed_task)

        relaxation_compilation = RelaxationCompilation(self.EXPSET.get_relaxed_tasks(), self.sas_task)
        if self.options.compile_relaxed_tasks:
            print("Compile relaxed tasks variables into planning task")
            relaxation_compilation.compile_tasks_part_one()

        # compile plan properties after relax task variables to have access to the relaxed task variables
        self.compile_plan_properties()

        if self.options.compile_relaxed_tasks:
            print("Compile relaxed tasks operators into planning task")
            relaxation_compilation.compile_tasks_part_two()

    def get_needed_facts(self):
        needed_facts = set()
        for relaxed_task in self.EXPSET.get_relaxed_tasks():
            for f in relaxed_task.get_init_fact_names() + relaxed_task.get_limits_fact_names():
                needed_facts.add(PredFact.parse(f))
        return needed_facts

    def get_needed_values(self, pre_sas_task):
        needed_facts = set()
        for relaxed_task in self.EXPSET.get_relaxed_tasks():
            for f in relaxed_task.get_init_fact_names() + relaxed_task.get_limits_fact_names():
                needed_facts.add(f)

        needed_vars = {}
        for fact in needed_facts:
            for var_id, variable in enumerate(pre_sas_task.variables.value_names):
                for value_id, value in enumerate(variable):
                    # print(fact + " " + value)
                    if 'Atom ' + fact == value:
                        if var_id not in needed_vars:
                            needed_vars[var_id] = set()
                        for i in range(0, len(variable)):
                            needed_vars[var_id].add(i)
                        break
        return needed_vars

    def compile_plan_properties(self):
        for name, s in self.EXPSET.action_sets.items():
            action_set_comp.compileActionSet(self.sas_task, s)

        AS_property.compileActionSetProperties(self.sas_task, self.EXPSET.get_action_set_properties(),
                                               self.EXPSET.action_sets)


        # TODO self.options.only_add_LTL_prop_to_SAS necessary
        final_syn_goal = LTL_property.compileLTLProperties(False, self.sas_task,
                                          self.EXPSET.get_ltl_properties(),
                                          self.EXPSET.action_sets)
        if final_syn_goal:
            self.EXPSET.add_goal_property(final_syn_goal)
            self.EXPSET.add_hard_goal(Goal(final_syn_goal.name))

        G_property.compileGoalProperties(self.sas_task, self.EXPSET.get_goal_properties())

        set_goals(self.sas_task, self.EXPSET, self.options)

        return True
