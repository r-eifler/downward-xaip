from sas_tasks import *

def addEntailmentsToTask(sas_task, propertyCollection):
    for p1 in propertyCollection.properties :
        pairs = [(p1.var_id, 1)]
        for p2 in propertyCollection.properties :
            if p1 != p2 :
                pairs.append((p2.var_id, 1))
        sas_task.entail.pairs.append(pairs) 
