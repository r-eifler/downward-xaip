from .parameter_matcher import ParamMatcher

class Action:

    def __init__(self, name, string):
        self.name = name
        self.string = string
        self.params = []

    def copy(self):
        nAction = Action(self.name, self.string)
        for p in self.params:
            nAction.addParam(p)

        return nAction
   

    def addParam(self, param):
        self.params.append(param.replace(" ", ""))

    @staticmethod
    def fromJSON(json, typeObjectMap):
        newAction = Action(json["name"], None)

        params = []
        if 'params' in json:
            params = json["params"];
        if 'args' in json:
            params = json["args"];

        for param in params:

            # if the param is a type instantiate the action with each object of the corresponding type
            # it does not matter if any of the combinations does not exists in the planning task
            if param in typeObjectMap:
                newAction.addParam("*")

            else:
                # if the param is an object just use the object
                # check if the object exists in the planning instance
                found = False
                for (o_type, objects) in typeObjectMap.items():
                    if param in objects:
                        found = True
                        break
                assert found, "Param: " + param + " of action " + newAction.name + " not found in planning task."

                newAction.addParam(param)

        return newAction

    def __repr__(self):
        s = "(" + self.name.lower() + " "
        for i in range(len(self.params)):
            s += self.params[i]
            if i < len(self.params) - 1:
                s+= " " 
        s += ")"

        return s


class ActionSet:

    def __init__(self, name, state_set):
        self.name = name
        self.actions = []
        self.action_dict = {}
        self.var_id = None
        self.state_set = state_set

        self.number_of_contained_ops = 0


    def addAction(self, a):
        self.actions.append(a)
        if not a.name in self.action_dict:
            self.action_dict[a.name] =  ParamMatcher()
        self.action_dict[a.name].addAction(a)


    def containsAction(self, action):
        parts = action.name.replace("(","").replace(")","").split()
        if parts[0] in self.action_dict:
            if self.action_dict[parts[0]].match(parts[1:]):
                self.number_of_contained_ops += 1
                #print(action.name)
                return True
        return False

    def intersect(self, other):
        my_actions = set(self.actions)
        other_actions = set(other.actions)
        return my_actions & other_actions


    def genSetDefinition(self):
        #TODO 
        return None

    @staticmethod
    def fromJSON(json, typeObjectMap, state_set):
        newActionSet = ActionSet(json["name"], state_set)
        for json_action in json["actions"]:
            newActionSet.addAction(Action.fromJSON(json_action, typeObjectMap))
        return newActionSet


    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)

    def __repr__(self):
        s = "***************************\n"
        s += self.name + "\n"
        s += "state_set: " + str(self.state_set) + "\n"
        s += "var_id: " + str(self.var_id) + "\n"
        s += "Actions:\n"
        for a in self.actions:
            s += str(a) + "\n"
        s += "***************************\n"

        return s
