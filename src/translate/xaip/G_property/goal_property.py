import xaip.logic.logic_formula as logic_formula
from xaip.general.property import PlanProperty
from xaip.action_sets.action import ActionSet


class GoalProperty(PlanProperty):

    def __init__(self, name, formula):
        super().__init__(name, formula)


    @staticmethod
    def fromJSON(json, typeObjectMap):
        formula = json['formula']
        new_property = GoalProperty(json['name'], formula)
        return new_property

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)
