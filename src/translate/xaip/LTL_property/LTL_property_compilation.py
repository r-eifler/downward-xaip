from sas_tasks import *
import xaip.logic.logic_formula as logic_formula
from .automata import State
import os
import copy

auxillary_vars = {}

class AuxillaryVariable:

    def __init__(self, constant, sas_task):
        self.name = "aux_" + constant.name
        self.id = len(sas_task.variables.value_names)
        self.domain = ["false_" + self.name, "true_" + self.name]

        sas_task.variables.value_names.append(self.domain)
        sas_task.variables.ranges.append(len(self.domain))
        sas_task.variables.axiom_layers.append(-1)
        
        #print(self.name)
        (var_id, value_id) = literalVarValue(sas_task, constant, False)[0]

        eff_false_con = (self.id, -1, 0, [])
        eff_true_con = (self.id, -1, 1, [])

        for o in sas_task.operators:
            for (id, pc, e, ce) in o.pre_post:
                if id == var_id:
                    if e == value_id:
                        o.pre_post.append(eff_true_con)
                    else:
                        o.pre_post.append(eff_false_con) 

        #init state: 
        if sas_task.init.values[var_id] == value_id:
            sas_task.init.values.append(1)
        else:
            sas_task.init.values.append(0)

# adds the fluents which describe the state of automata
def addFluents(automata, id, sas_task):
    states_sorted = list(automata.get_states())
    #sort the states according to their id TODO which ID ?
    states_sorted.sort(key=State.get_name)

    variable = []
    #if the automata has more than one state one value per state is added
    if len(states_sorted) > 1:
        for s in states_sorted:
            variable.append("at(" + s.name + "," + automata.name + ")")
    else:
        #if the automata has only one state you can be either in this state or not 
        # fast downward variables have to have at least a domain size of 2
        for s in states_sorted:
            variable.append("at(" + s.name + "," + automata.name + ")")
            variable.append("not_at(" + s.name + "," + automata.name + ")")

    # id of the position var in the encoding of the variables
    automata.pos_var = len(sas_task.variables.value_names)

    #add variables to task
    sas_task.variables.value_names.append(variable)
    sas_task.variables.ranges.append(len(variable))
    sas_task.variables.axiom_layers.append(-1)

    # at to the initial state the initial state of the automata
    sas_task.init.values.append(automata.init.id)

    #variable to indicate if the automata is currently in an accepting state -> later used in the "goal fact dependencies"
    # id of the accepting var in the encoding to the variables 
    automata.accept_var = len(sas_task.variables.value_names)
    accept_var_domain = ["not_accepting(" + automata.name +")", "soft_accepting(" + automata.name + ")"]
    sas_task.variables.value_names.append(accept_var_domain)
    sas_task.variables.ranges.append(len(accept_var_domain))
    sas_task.variables.axiom_layers.append(-1)

    #add to the initial state of the planning task if the initial state of the automata is accepting
    sas_task.init.values.append(int(automata.init.accepting))
    #in the goal the automata has to be in an accepting state
    #sas_task.goal.pairs.append((automata.accept_var, 1))


#add the transitions of the automata to the planning task
def automataTransitionOperators(automata, sas_task, actionSets):
    #print("----------------------------------------------------------")
    new_operators = []
    for name, s in automata.states.items():
        for t in s.transitions:
            #print(t)

            #print("Process action: " + t.name) 

            #encoding of the precondition and effect (var, pre, post, cond)
            pre_post = []
                
            #automata state: from state source to state target
            pre_post.append((automata.pos_var, t.source.id, t.target.id, []))
            #print("Var: " + str(automata.pos_var) + " Source: " + str(t.source.id) + " -> Target: " + str(t.target.id))

            #accepting
            # if the target state is accepting than the variable which indicates the acceptance of the
            # automata is set to true
            pre_post.append((automata.accept_var, int(t.source.accepting), int(t.target.accepting), []))
            #print("Var: " + str(automata.accept_var) + " Source: " + str(t.source.accepting) + " -> " + str(t.target.accepting))

            #transition name:
            t_name = automata.name + ": " + t.name
            #encode the guard of the transition in the precondition of the action
            if not t.guard.isTrue():
                # returns a disjunction of pre_post, such that every pre_post belongs to one action
                clauses = t.getClauses()
                #print("--------------------------------------------------------")
                #print(t.guard)
                #print("Clauses:")
                #print(clauses)
                #print("-----")
                #clauses = remove_unnecessary_clauses(clauses, sas_task, actionSets)
                #print("Simplified: ")
                #print(clauses)

                for clause in clauses:
                    con = []
                    # for every literal find the corresponding variable
                    for l in clause:
                        new_con = []

                        # action set variables
                        if l.constant.name in actionSets:
                            if l.negated:
                                new_values = [(actionSets[l.constant.name].var_id, 0)]
                            else:
                                new_values = [(actionSets[l.constant.name].var_id, 1)]
                        # world variables
                        else:
                            new_values = literalVarValue(sas_task, l.constant, l.negated)
                            assert new_values, "Constant " + str(l.constant) + " does not exist. " + str(sas_task.variables.value_names)
                            # check if auxiliary variable is necessary
                            if l.negated and len(new_values) > 1:
                                if l.constant.name not in auxillary_vars:
                                    # create new auxiliary var
                                    new_aux_var = AuxillaryVariable(l.constant, sas_task)
                                    auxillary_vars[l.constant.name] = new_aux_var

                                aux_var = auxillary_vars[l.constant.name]
                                new_values = [(aux_var.id, 0)]

                        # make sure that a a variable for the literal was found
                        assert new_values and len(new_values) > 0, "value not found: " + str(l.constant)

                        # from var, value to pre_post tuple
                        for var, value in new_values:
                            con.append((var, value, value, []))

                    # combine the conditions of the guard with the state transition and accepting condition
                    final_pre_post = pre_post + con     # TODO check if this is still correct
                    #print(final_pre_post)
                    op = SASOperator(t_name,[], final_pre_post, 0)
                    new_operators.append(op)
            else:
                # if the guard is true no other conditions are necessary
                #print("Guard True")
                new_operators.append(SASOperator(t_name,[], pre_post, 0))

    #print("Num operators: " + str(len(new_operators)))
    return new_operators


# TODO equ for literals
def remove_unnecessary_clauses(clauses, sas_task, actionSets):
    # TODO also implement for world variables
    result_clauses = []
    for clause in clauses:
        num_pos_literals = 0
        result_clause = copy.copy(clause)
        # remove negative literals which are implied by positive once
        for literal in result_clause:
            if not literal.negated and literal.constant.name in actionSets:
                num_pos_literals += 1
                #print("_________")
                #print("Checking: " + str(literal))
                # check if any of the negative literals is implied
                for comp_literal in result_clause:
                    #print("Next: " + str(comp_literal))
                    if literal != comp_literal and comp_literal.negated and comp_literal.constant.name in actionSets:
                        #print("Check Implication with: " + str(comp_literal))
                        # only if the intersection is empty literal -> comp_literal
                        intersection = actionSets[literal.constant.name].intersect(actionSets[comp_literal.constant.name])
                        #print(intersection)
                        if len(intersection) == 0:
                            result_clause.remove(comp_literal)
                            #print("Remove")
                            #print(result_clause)
                            break
        result_clauses.append(result_clause)
        index1 = 0
        for rc in result_clauses:
            index2 = 0
            for comp_rc in result_clauses:
                if index1 != index2 and set(rc) == set(comp_rc):
                    result_clauses.remove(comp_rc)
                index2 += 1
            index1 += 1
    return result_clauses



#add to every action in the original planning task the syn variable as precondition
# it has to be false when an actions is executed and is true afterwads
def addSyncPreconditionEffects(pre_sync_var, eff_sync_var, operators):

    pre_sync_con = (pre_sync_var, 1, 0, [])
    eff_sync_con = (eff_sync_var, 0, 1, [])

    for o in operators:
        o.pre_post.append(pre_sync_con)
        o.pre_post.append(eff_sync_con)
            
            
# computes for a literal the variable and value id in the planning task encoding
# neg indicates if the literal is used in a negated context
#TODO
def literalVarValue(sas_task, constant, neg):
    #print("Literal: " + str(literal))
    # the literal has to be mapped from "l_id" back to e.g. "ontable(a)" to be able to find it
    for var in range(len(sas_task.variables.value_names)):
        values = sas_task.variables.value_names[var]
        for v in range(len(values)):
            value_name = values[v].replace(", ", ",")
            if "Atom " + constant.name == value_name:
                if neg:
                    #print(len(values))
                    #if the domain size of the variable is larger than 2, return all other variables except the given one
                    if(len(values) > 2):
                        res = []
                        for i in range(len(values)):
                            if i != v:
                                res.append((var, i))
                        return res
                    else:
                        # if the domain size is 2, return the negated constant
                        return [(var, v+1)]
                return [(var, v)]

    return None

def addWorldSyncvar(sas_task, properties):
    world_sync_var = len(sas_task.variables.value_names)
    sync_var_domain = ["sync(world)"]
    for p in properties:
        sync_var_domain.append("sync(" + p.automata.name + ")")

    sas_task.variables.value_names.append(sync_var_domain)
    sas_task.variables.ranges.append(len(sync_var_domain))
    sas_task.variables.axiom_layers.append(-1)

    #initially we are in a wold state
    #sas_task.init.values.append(0)
    #initially we evaluate the first automaton in the initial state
    sas_task.init.values.append(1)

    return world_sync_var

def add_sync_conditions(sas_task, operators, properties, world_sync_var):

    operators_phases = [sas_task.operators] + operators

    #add the synchronization precondition and effects to the different operator sets
    for i in range(len(properties)+1):

        sync_con = (world_sync_var, i, (i+1)%(len(properties)+1), [])

        for o in operators_phases[i]:
            o.pre_post.append(sync_con)



def add_reset_state_sets(last_ops, actionSets):
    for s in actionSets.values():
        if s.state_set:
            for op in last_ops:
                op.pre_post.append((s.var_id, -1, 0, []))

    
def compileLTLProperties(only_add_to_SAS, sas_task, properties, actionSets):

    if len(properties) == 0:
        return 

    new_operators = []


    # only compile action sets into task (is done at some other point)
    # adds the properties into the SAS file
    if only_add_to_SAS:
        print("Don't encode LTL properties into task.")
        for i, p in enumerate(properties):
            name, f = p.SAS_repr(actionSets)
            sas_task.addLTLProperty(name, f)
    else:
        # generate automaton representation for each ltl property
        for prop in properties:
            prop.generateAutomataRepresentation()

        # compile properties into the task
        print("Encode LTL properties into task.")

        #each automata and the world itself have a synchronization variable to constrain the execution order
        world_sync_var = addWorldSyncvar(sas_task, properties)

        for i, p  in enumerate(properties):
            automata = p.automata
            #print(str(a))
            addFluents(automata, i, sas_task) #also addd the sync vars
            new_operators.append(automataTransitionOperators(automata, sas_task, actionSets))
            p.var_id = automata.accept_var; # the property is excepted if the automata excepts it


        add_sync_conditions(sas_task, new_operators, properties, world_sync_var)
        add_reset_state_sets(new_operators[len(properties)-1], actionSets)

        for ol in new_operators:
            for o in ol:
                sas_task.operators.append(o)

        print("Number of auxiliary variables: " + str(len(auxillary_vars)))




