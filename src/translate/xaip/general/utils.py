# computes for a literal the variable and value id in the planning task encoding
# neg indicates if the literal is used in a negated context
def literalVarValueList(sas_task, constant, neg):
    constant = constant.lower()
    constant = constant.replace(" ", "")
    # the literal has to be mapped from "l_id" back to e.g. "ontable(a)" to be able to find it
    for var in range(len(sas_task.variables.value_names)):
        values = sas_task.variables.value_names[var]
        for v in range(len(values)):
            value_name = values[v].replace(" ", "")
            if "Atom" + constant == value_name or "soft_" + constant == value_name or constant == value_name:
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


def literalVarValue(sas_task, constant):
    constant = constant.lower()
    constant = constant.replace(" ", "")
    # the literal has to be mapped from "l_id" back to e.g. "ontable(a)" to be able to find it
    for var in range(len(sas_task.variables.value_names)):
        values = sas_task.variables.value_names[var]
        for v in range(len(values)):
            value_name = values[v].replace(" ", "")
            if "Atom" + constant == value_name or "soft_" + constant == value_name or constant == value_name:
                return var, v

    return None, None