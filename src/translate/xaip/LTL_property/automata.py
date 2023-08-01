import sys
import re
import xaip.logic.logic_formula as logic_formula

#automata state
class State:

    def __init__(self, name):
        self.name = name
        #is initial state
        self.isInit = False
        #is accepting state
        self.accepting = False
        #outgoing transitions
        self.transitions = []

    def get_name(self):
        return self.name

    def isBlocking(self):
        return len(self.transitions) == 1 and not self.transitions[0].guard.isTrue()

    def replaceConstantsName(self, map):
        for t in self.transitions:
            t.replaceConstantsName(map)

    def  __repr__(self):
        s = self.name + ":\n"
        for t in self.transitions:
            s += str(t) + "\n"

        return s

def compStates(s1, s2):
    if s1.id < s2.id:
        return -1
    if s1.id > s2.id:
        return 1

    return 0



class Transition:
    
    def __init__(self, name, source, target, guard):
        self.name = name
        self.source = source
        self.target = target
        self.guard = guard

    def replaceConstantsName(self, map):
        self.guard = self.guard.replaceConstantsName(map)
        self.name = "(" + self.source.name + " (" + str(self.guard) + ") -> " + self.target.name + ")"

    def getClauses(self):
        if(isinstance(self.guard, logic_formula.LOr)):
            return self.guard.getClauses([])
        else:
            return [self.guard.getClauses([])]

    def  __repr__(self):
        return "\t (" + str(self.guard) + ") -> " + self.target.name


class Automata:

    def __init__(self, name):
        # use to assign an unique id to every state
        self.state_ids = 0
        self.name = name
        #initial state
        self.init = None
        # maps state name to state
        self.states = {}
        self.transitions = []


    def get_states(self):
        return self.states.values()


    def addState(self, s):
        if(s.name in self.states):
            return

        #assign unique id
        s.id = self.state_ids
        self.state_ids += 1

        #store state
        self.states[s.name] = s


    def replaceConstantsName(self, map):
        for name, s in self.states.items():
            s.replaceConstantsName(map)

    def eliminateBlockingStates(self):
        for name, s in self.states.items():
            if s.isBlocking():
                dummy_state = State("dummy_" + name)
                dummy_state.accepting = False
                self.addState(dummy_state)
                #transition to dummy state
                neg_trans = Transition("neg_trans", s, dummy_state, logic_formula.LNot(s.transitions[0].guard))
                s.transitions.append(neg_trans)

                #self loop dummy state
                skip_trans = Transition("dummy_trans", dummy_state, dummy_state, logic_formula.LConstant("true", 0))
                dummy_state.transitions.append(skip_trans)
            
    def  __repr__(self):
        string = "______________________\nAutomata: " + self.name + "\n"
        for name, s in self.states.items():
            string += str(s)
        string += "___________________________\n"
        return string

    






