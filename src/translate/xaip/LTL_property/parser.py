import xaip.logic.logic_formula as logic_formula
from .LTL_property import LTLProperty

def parse_LTL_property(lines):

    assert(len(lines) == 2)
    line = lines[0]
        
    name = line.split(" ")[1].replace('\n','')

    parts = lines[1].split()
    #print(parts)
    (formula, rest, c) = logic_formula.Operator.parse(parts)
    #TODO constants
 
    assert len(rest) == 0, "Parse LTL property " + name + " failed:\n\tresult: " + str(formula) + "\n\trest is not empty: " + str(rest)

    return LTLProperty(name, formula, c)
