from xaip.general.utils import literalVarValue

class Goal:

    def __init__(self, var_name, var_id=None, value=None):
        self.var_name = var_name
        self.var_id = var_id
        self.value = value

    def get_sas_fact(self, sas_task, EXPSET):

        if self.var_id is not None and self.value is not None:
            return self.var_id, self.value
        if self.var_name:
            prop = EXPSET.get_property(self.var_name)
            if prop:
                self.var_id = prop.var_id
                self.value = prop.var_sat_goal_value
            else:
                self.var_id, self.value = literalVarValue(sas_task, self.var_name)
            assert self.var_id is not None and self.value is not None, "ERROR: invalid goal: " + self.var_name
            return self.var_id, self.value

    def __eq__(self, other):
        return self.var_name == other.var_name or (self.var_id and other.var_id  and self.var_id == other.var_id)

    def __hash__(self):
        return hash(self.var_name)

    @staticmethod
    def fromJSON(json, EXPSET):
        goals = []
        for goal_name in json:
            new_goal = Goal(goal_name)
            goals.append(new_goal)
        return goals

    def __repr__(self):
        return self.var_name + ": " + str(self.var_id) + "=" + str(self.value)


# adds the hard and soft goals as specified in the explanation setting and
# checks that the overall goals only contains the facts which are either in
# the soft or in the hard goals
def set_goals(sas_task, EXPSET, options):
    print("****************************************************************************")
    if EXPSET.has_hard_goals() or EXPSET.has_soft_goals():
        print("hard and soft goals are specified")
        # add hard goals
        for hg in EXPSET.hard_goals:
            sas_task.addHardGoalFact(hg.get_sas_fact(sas_task, EXPSET))

        # add soft goals
        for sg in EXPSET.soft_goals:
            sas_task.addSoftGoalFact(sg.get_sas_fact(sas_task, EXPSET))

        # make goals consistent with hard + soft goals
        # if options.all_goals_as_sas_goal_facts :
        sas_task.goal.reset_facts([g.get_sas_fact(sas_task, EXPSET) for g in EXPSET.hard_goals + EXPSET.soft_goals])
        # else:
        #     sas_task.goal.reset_facts([g.get_sas_fact(sas_task, EXPSET) for g in EXPSET.hard_goals])

    else:
        print("original goal facts are hard goals and all properties are soft goals")
        print("(accept if indicated differently with all_soft_goals=True)")
        # original goal facts are hard goals and all properties are soft goals
        for gf in sas_task.goal.pairs:
            sas_task.addHardGoalFact(gf)

        for pg in EXPSET.get_action_set_properties() + EXPSET.get_ltl_properties() + EXPSET.get_goal_properties():
            pair = Goal(pg.name).get_sas_fact(sas_task, EXPSET)
            sas_task.addSoftGoalFact(pair)
            sas_task.goal.pairs.append(pair)
