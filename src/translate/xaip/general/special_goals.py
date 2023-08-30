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
    # if there are hard goals then they acan be necessary for the LTLf compilation
    # so they need to stay hardgoals
    
    soft_goals = []
    
    if EXPSET.has_hard_goals() and EXPSET.has_soft_goals():
        print("hard AND soft goals are specified (goals not specified as hard or soft goals are not considered)")
        # add hard goals
        for hg in EXPSET.hard_goals:
            sas_task.addHardGoalFact(hg.get_sas_fact(sas_task, EXPSET))

        # add soft goals
        for sg in EXPSET.soft_goals:
            soft_goals.append(sg)
            sas_task.addSoftGoalFact(sg.get_sas_fact(sas_task, EXPSET))

        # make goals consistent with hard + soft goals
        sas_task.goal.reset_facts([g.get_sas_fact(sas_task, EXPSET) for g in EXPSET.hard_goals + EXPSET.soft_goals])
    
    if EXPSET.has_hard_goals():
        print("hard goals are specified but not soft goals -> original goal facts and all remaining properties are soft goals")
        
        # add original goals as soft goals
        for gf in sas_task.goal.pairs:
            sas_task.addSoftGoalFact(gf)
        
        # add hard goals
        hard_goal_names = []
        for hg in EXPSET.hard_goals:
            hard_goal_names.append(hg.var_name)
            pair = hg.get_sas_fact(sas_task, EXPSET)
            sas_task.addHardGoalFact(pair)
            sas_task.goal.pairs.append(pair)

        # add properties as soft goal
        for pg in (EXPSET.get_action_set_properties() + EXPSET.get_ltl_properties() + EXPSET.get_goal_properties()):
            if pg.name in hard_goal_names:
                continue
            pair = Goal(pg.name).get_sas_fact(sas_task, EXPSET)
            sas_task.addSoftGoalFact(pair)
            sas_task.goal.pairs.append(pair)
            soft_goals.append(pg)

    else:
        print("No hard and no soft goals -> original goal facts and all properties are soft goals")
        for gf in sas_task.goal.pairs:
            sas_task.addSoftGoalFact(gf)

        for pg in EXPSET.get_action_set_properties() + EXPSET.get_ltl_properties() + EXPSET.get_goal_properties():
            pair = Goal(pg.name).get_sas_fact(sas_task, EXPSET)
            sas_task.addSoftGoalFact(pair)
            sas_task.goal.pairs.append(pair)
            soft_goals.append(sg)
            
            
    # update soft goal graph
    edges = set()
    for sg in soft_goals:
        sg_pair = Goal(sg.name).get_sas_fact(sas_task, EXPSET)
        for w in sg.weaker:
            edges.add((sg_pair, Goal(w).get_sas_fact(sas_task, EXPSET)))
        for s in sg.stronger:
            edges.add((Goal(s).get_sas_fact(sas_task, EXPSET), sg_pair))
    
    print('------ soft goal graph ----')
    print('num edges: ' + str(len(edges)))
    for e in edges:
        print(e)
    sas_task.addSoftGoalGraph(list(edges))