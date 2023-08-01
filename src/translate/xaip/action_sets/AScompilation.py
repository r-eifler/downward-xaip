
def compileActionSet(sas_task, actionSet):
    addActionSetIndicators(sas_task, actionSet)
    updateOriginalActions(sas_task, actionSet)

def addActionSetIndicators(sas_task, s):

    #for every action set add an variable which indicates if one of the actions in the set has been used  
    new_vars =  ["not_used_" + s.name, "used_" + s.name]

    s.var_id = len(sas_task.variables.value_names) # store id of the variable in the action set
    sas_task.variables.value_names.append(new_vars)
    sas_task.variables.ranges.append(len(new_vars))
    sas_task.variables.axiom_layers.append(-1)

    # update initial state
    sas_task.init.values.append(0) # at the beginning of the planning task no action has been used


def updateOriginalActions(sas_task, s):
    # every action in the set assigns the corresponsing variable to true
    #print("-------------------------------------------------------------------")
    #print(s)
    for op in sas_task.operators:   
        #print(op.name)     
        #if the action is in the action set than is assigns the variable to True/Used              
        if s.containsAction(op):
            #print(op.name)
            op.pre_post.append((s.var_id, -1, 1, []))    

    #assert s.number_of_contained_ops > 0, "WARNING: " + s.name + " does not contain any action"
    if s.number_of_contained_ops == 0:
        print("WARNING: " + s.name + " does not contain any action")
        print(s)