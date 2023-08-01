from . import parser
from . import action_set_property_compilation

def addActionSetPropertiesToTask(path, task, sas_task, options, addGoalFacts, addNegSatActions):
    

    # parse action sets and properties
    asps = parser.parseActionSetProperty(path, typeObjectMap)

    #compile properties into sas_task

    print("property_compilation_type: " + str(options.property_compilation_type))
    if options.property_compilation_type == None or options.property_compilation_type == 0:
        action_set_property_compilation.compileToTask(sas_task, asps, addPropertiesToGoal=addGoalFacts, addNegativeSatActions=addNegSatActions)
       
        return asps

    if options.property_compilation_type == 1:
        print("Properties folder: " + options.properties_folder)
        asps.generateImpPropertyFiles(options.properties_folder)
        return asps



    


