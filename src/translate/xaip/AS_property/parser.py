from xaip.action_sets.action import *
from .action_set_property import *


def parse_action_set_property(lines):
    assert(len(lines) == 2)
    line = lines[0]

    soft = line.split()[0].startswith("soft")
    #name of the property
    name = line.split()[1]

    line = lines[1]
         
    #parse prefix notion logic formula
    property_string = line.replace("\n","")
    (formula, rest, constants) = logic_formula.parseFormula(property_string)
    asProperty = ActionSetProperty(name, formula, constants)

    return asProperty

