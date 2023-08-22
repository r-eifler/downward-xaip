from .action import Action, ActionSet


def parse_action(string, typeObjectMap):
    parts = string.split()
    actionName = parts[0]
    action = Action(actionName, string)

    for n in range(1, len(parts)):
        #remove whitespaces
        param = parts[n].replace(" ","")
        if param == " " or param == "" or param == "\t" or param == "\n":
                continue


        #if the param is a type instantiate the action with each object of the corresponsing type
        #it does not matter if any of the combinations does not exists in the planning task
        if param in typeObjectMap:              
            action.addParam("*")

        else:
            # if the param is an object just use the object
            #check if the object exists in the planning instance
            found = False
            for (o_type, objects) in typeObjectMap.items():
                if param in objects:
                    found = True
                    break
            assert found, "Param: " + param + " of action " + string + " not found in planning task."
            
            action.addParam(param)

    return action

def parse_action_set(lines, typeObjectMap):
    line = lines.pop(0)
    #print("Actionset: " + line)
    set_parts = line.replace("\n","").split()
    
    name = set_parts[1]
    state_set = set_parts[0].startswith("state")
    number = int(set_parts[2])
    newActionSet = ActionSet(name, state_set)

    while len(lines) > 0:

        line = lines.pop(0)
        #print(line)

        # ignore comment and empty lines
        if line.startswith("#"):
            continue

        actionString = line.replace("\n","")
        newActionSet.addAction(parse_action(actionString, typeObjectMap))               

    return newActionSet