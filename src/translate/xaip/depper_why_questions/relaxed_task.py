
def find_var_value_id(sas_variables, fact):
    negated = fact.startswith('!')
    pure_fact = fact.replace('! ','')
    for var_id, variable in enumerate(sas_variables):
        for value_id, value in enumerate(variable):
            if (negated and 'NegatedAtom ' + pure_fact == value) or (not negated and 'Atom ' + pure_fact == value):
                return var_id, value_id
    #return None
    #print('Warning: ' + fact + ' not in ' + str(sas_variables))
    assert False, fact + ' not in ' + str(sas_variables)


def parse_tasks(json_def):
    return [RelaxedTask.parse(tj) for tj in json_def]


class RelaxedTask:

    def __init__(self, id, name):
        self.id = id
        self.name = name
        self.init = []
        self.init_fact_names = []
        self.limits = []
        self.limits_fact_names = []
        self.limit_type = 'OR'

        self.upper_cover = []
        self.lower_cover = []

    def get_init_fact_names(self):
        return [f.replace('! ','') for f in self.init_fact_names]

    def get_limits_fact_names(self):
        return [f.replace('! ','') for f in self.limits_fact_names ]

    def __repr__(self):
        return str(self.id) + ': ' + self.name + '\n ' + str(self.init) + '\n ' + str(self.limits) + '\n ' + str(self.cover)

    def update_init_and_limits(self, sas_task):
        var_names = sas_task.variables.value_names
        self.init = [find_var_value_id(var_names,f) for f in self.init_fact_names]
        self.limits = [find_var_value_id(var_names,f) for f in self.limits_fact_names]

    @staticmethod
    def parse(json_def):
        print(json_def)
        assert ('name' in json_def and 'id' in json_def and 'upper_cover' in json_def and 'lower_cover' in json_def)
        assert ('inits' in json_def or 'limits' in json_def)

        id = json_def['id']
        name = json_def['name']

        newTask = RelaxedTask(id, name)

        newTask.lower_cover = json_def['lower_cover']
        newTask.upper_cover = json_def['upper_cover']

        if 'inits' in json_def:
            newTask.init_fact_names = json_def['inits']
        else:
            newTask.init_fact_names = []

        if 'limits' in json_def:
            limits = json_def['limits']
            newTask.limits_fact_names = limits['facts']
            newTask.limit_type = limits['type']
        else:
            newTask.limits_fact_names = []
            newTask.limit_type = []

        return newTask




