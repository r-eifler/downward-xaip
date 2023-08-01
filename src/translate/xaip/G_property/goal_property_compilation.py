from xaip.general.utils import literalVarValue

def compileGoalProperties(sas_task, properties):
    for prop in properties:
        var, value = literalVarValue(sas_task, prop.formula)
        assert var is not None and value is not None, "ERROR: invalid goal: " + prop.name + ' #variables: ' + str(sas_task.variables.value_names)
        prop.var_id = var
        prop.var_sat_goal_value = value