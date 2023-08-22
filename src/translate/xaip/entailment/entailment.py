import xaip.logic.logic_formula as logic

class Property:

    def __init(self, formula):
        self.formula = formula
        self.var_id = None
        self.var_value = None

class BasicProperty(Property):

    def __init(self, formula):
        super(BasicProperty, self).__init__(formula)

    def compileEvalToTask(sas_task, )
    

class ComposedProperty(Property):

    def __init(self, formula):
        super(ComposedProperty, self).__init__(formula)

   


class Entailment: 

    def __init__(self, premise):
        self. premise = premise
        self.conclusions = []

    def addConclusion(self, con):
        self.conclusions.append(con)


    

        
