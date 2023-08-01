import xaip.logic.logic_formula as logic_formula
from sas_tasks import *


#compile the action sets properties into the planning task


def addChangePhase(sas_task):

    #variable to change from execution to one eval phase per property
    eval_phase_var_id = len(sas_task.variables.value_names)
    eval_var = [] 
    for i in range(2): 
        eval_var.append("phase_" + str(i))

    sas_task.variables.value_names.append(eval_var)
    sas_task.variables.ranges.append(len(eval_var)) #binary var
    sas_task.variables.axiom_layers.append(-1)

    #start with not eval phase
    sas_task.init.values.append(0)

    for op in sas_task.operators:
        #original operators can only be executed in the execution phase
        op.pre_post.append((eval_phase_var_id, 0, 0, []))

    #action which changes from execution to evaluation phase    
    pre_post_change_phase = []
    pre_post_change_phase.append((eval_phase_var_id, 0, 1, []))
    sas_task.operators.append(SASOperator("(change_to_evaluation)",[], pre_post_change_phase, 0))

    

    return eval_phase_var_id


def addPropertySatVariables(sas_task, prop, addToGoal=False):
    # compile property into one (goal) fact 
    #fact that indicates if the property is satisfied at the end of the plan
    prop.var_id = len(sas_task.variables.value_names)
    # TODO implete separate handling of soft and hard goal identification
    #soft_prefix = "hard"
    # if prop.soft:
    #     soft_prefix = "soft"
    #prop_vars = [soft_prefix + "_not_sat_" + prop.name, soft_prefix + "_sat_" + prop.name]
    prop_vars = ["not_sat_" + prop.name, "sat_" + prop.name]

    #print(prop.name + " : " + str(prop.var_id))

    sas_task.variables.value_names.append(prop_vars)
    sas_task.variables.ranges.append(len(prop_vars)) #binary var
    sas_task.variables.axiom_layers.append(-1)

    # update initial state -> at the beginning the property is not satisfied
    sas_task.init.values.append(0)

    # if addTo:
        # update goal -> at the end the property has to be satisfied
        # sas_Goaltask.goal.pairs.append((prop.var_id, 1))




def addPropertyCheckingActions(sas_task, prop, actionSets, eval_phase_var_id):
    #add the actions checking if the property ist true
    # Assumption: property is in disjunctive normal form

    #print("Clauses: \n Property: " + str(prop))
    #for every clause in DNF we have to create one action
    clauses = prop.getClauses()
    #print(clauses)
    for c in clauses:
        pre_post = []
        # literals form the preconditions
        for l in c:
            assert l.constant.name in actionSets, "action set " + l.constant.name + " does not exist."
            if l.negated:
                pre_post.append((actionSets[l.constant.name].var_id,0,0,[]))
            else:
                pre_post.append((actionSets[l.constant.name].var_id,1,1,[]))
            #print(pre_post)

        # as effect the property fact is set to true
        pre_post.append((prop.var_id, -1, 1, []))

        #can only be executed in its evaluation phase
        pre_post.append((eval_phase_var_id, 1, 1, []))

        #TODO assigning a cost of 0 does not work -> only possible in a pddl version which uses action costs
        
        #print(pre_post)
        sas_task.operators.append(SASOperator("(" + prop.name + "_ " + str(c) + ")",[], pre_post, 0))


def specifySoftGoals(sas_task, asp):
     #specify soft goals
    for g in asp.soft_goals:
        #print("Add soft_goal: " + g)
        for i in range(len(sas_task.variables.value_names)):
            for j in range(len(sas_task.variables.value_names[i])):
                value_name = sas_task.variables.value_names[i][j]
                #print("\t " + value_name)
                if value_name.endswith(g):
                    sas_task.variables.value_names[i][j] = "soft_" + value_name


def compileActionSetProperties(sas_task, properties, actionSets, addPropertiesToGoal=False):

    if len(properties) == 0:
        return

    eval_phase_var_id = addChangePhase(sas_task)

    for prop in properties:
        addPropertySatVariables(sas_task, prop, addPropertiesToGoal)
        addPropertyCheckingActions(sas_task, prop, actionSets, eval_phase_var_id)

    ####################### GOALS ########################
    #specifySoftGoals(sas_task, asp)
        
        

       

       