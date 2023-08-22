from .automata import *
import xaip.logic.logic_formula as logic_formula
import re


def parseGuard(guard_string):

    #print("Parse guard: ")  
    guard_string = guard_string.replace("(", " ( ").replace(")", " ) ")
    #print(guard_string)
    parts = logic_formula.shuntingYard(guard_string.split())
    #print(parts)
    guard, rest, constants = logic_formula.Operator.parse(parts)
    assert len(rest) == 0, "Parse LTL property failed:\n\tresult: " + str(guard) + "\n\trest is not empty: " + str(rest)

    return guard, constants

def parseTransition(line, automata, source):
    #print("Trans: " + line)
    m = re.match(r"[\s]*:: (.+) -> goto (.+)", line)
    assert(m)
    
    guard_string = m.group(1)
    guard, constants = parseGuard(guard_string)

    targetName = m.group(2)
    if not targetName in automata.states:
        target = State(targetName)
        automata.addState(target)
    else:
        target = automata.states[targetName]


    trans = Transition(str(guard) + "->" + target.name, source, target, guard)
    automata.transitions.append(trans)
    source.transitions.append(trans)

    return constants

def parseStateName(line, automata):
    m = re.match("[\s]*(.+):", line)
    #print("State name",line)

    name = m.group(1)

    isInit = name.endswith("init")
    accepting = name.startswith("accept")

    if not name in automata.states:
        state = State(name)
        automata.addState(state)
    else:
        state = automata.states[name]

    state.isInit = isInit
    state.accepting = accepting

    if state.isInit:
        automata.init = state

    return state




def parseNFA(path):
    automata = Automata("test_automata")

    reader = open(path, "r")
    lines = reader.readlines()

    source = None
    constants = []

    while len(lines) > 0:
        line = lines.pop(0)
        line = line.replace("\t", "")
        line = line.replace("\n", "")

        if line.startswith("never") or line.startswith("}"):
            continue

        if re.match("[\s]+if", line):
            constants += parseTransition(lines.pop(0).replace("\t",""), automata, source)
            continue

        if re.match("[\s]+::", line):
            #print("Transition",line)
            constants += parseTransition(line, automata, source)
            continue

        if re.match("[\s]+fi", line):
            source = None
            continue

        # the skip action is interpreted as a "true" self loop 
        if re.match("[\s]+skip", line):
            true_exp = logic_formula.LConstant("true", None)
            trans = Transition("skip", source, source, true_exp)
            automata.transitions.append(trans)
            source.transitions.append(trans)
            continue

        source = parseStateName(line, automata)

    

    return automata    


def removeDuplicates(list):
    new_list = []
    for e in list:
        if e in new_list:
            continue
        new_list.append(e)