from .action_sets.parser import parse_action_set 
from .AS_property.parser import parse_action_set_property
from .LTL_property.parser import parse_LTL_property

# typeObjectMap maps from a type to a list of objects which have this type
def parse(path, typeObjectMap):

    reader = open(path, "r")
    lines = reader.readlines()
    reader.close()

    actionSets = {}
    AS_properties = []
    LTL_properties = []

    while len(lines) > 0:
        line = lines[0].replace("\n","") 


        # ignore comment and empty lines
        if line.startswith("#") or line == "\n" or line == "":
            lines.pop(0)
            continue

        # parse set: set <name> <number of elems>
        if line.startswith("set") or line.startswith("state_set"):
            set_def = lines_until_empty_line(lines)
            newActionSet = parse_action_set(set_def, typeObjectMap)
            assert newActionSet.name not in actionSets, "Not unique action set name: " + newActionSet.name
            actionSets[newActionSet.name] = newActionSet
            continue

        if line.startswith("AS_property ") or line.startswith("soft-AS_property ") or line.startswith("soft-property "):
            property_def = lines_until_empty_line(lines)
            asProperty = parse_action_set_property(property_def)
            AS_properties.append(asProperty)
            continue

        if line.startswith("LTL_property ") or line.startswith("soft-LTL_property "):
            property_def = lines_until_empty_line(lines)
            ltlProperty = parse_LTL_property(property_def)
            LTL_properties.append(ltlProperty)
            continue


        #TODO
        #if line.startswith("soft-goals"):
        #    lines.pop(0)
        #    line = lines.pop(0).replace("\n","")
        #    while line != "" and len(lines) > 0:
        #        actionSetProperties.soft_goals.append(line)
        #        line = lines.pop(0).replace("\n","")
        #    continue

        #print("nothing done")
        assert False, line

    #print(actionSetProperties)
    #print(">>>>>>>>>>>>>>>>>>>Parse finished>>>>>>>>>>>>>>>>>>>>>>>>>><")

    return (actionSets, AS_properties, LTL_properties)


def lines_until_empty_line(lines):
    selected_lines = []
    line = lines.pop(0)
    while line != "\n":
        selected_lines.append(line)
        line = lines.pop(0)
    return selected_lines