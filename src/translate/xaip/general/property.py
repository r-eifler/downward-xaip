
class PlanProperty:

    def __init__(self, name, formula, weaker=[], stronger=[]):
        self.name = name
        self.formula = formula
        self.actionSets = []
        # names of the set names that are used in the property
        self.constants = []
        # id of the sat variable in the SAS encoding
        self.var_id = None
        self.var_sat_goal_value = 1 # 1 means prop has to be satisfied
        # names of plan properties that are weaker/stronger
        self.weaker = weaker
        self.stronger = stronger

    def add_action_set(self, s):
        self.actionSets.append(s)

    def get_action_sets(self):
        return self.actionSets

    def add_constant(self, c):
        self.constants.append(c)
        
    def add_weaker(self, name):
        self.weaker.append(name)
        
    def add_stronger(self, name):
        self.stronger.append(name)

    def update_action_set_name(self, oldName, newName):
        constant_map = {oldName: newName}
        self.formula = self.formula.replaceConstantsName(constant_map)
        new_constants = []
        for c in self.constants:
            new_constants.append(constant_map[c] if c in constant_map else c)
        self.constants = new_constants

    def __repr__(self):
        s = self.name + ":\n"
        s += "\tformula:" + str(self.formula) + "\n"
        s += "\tvar_id:" + str(self.var_id)
        return s