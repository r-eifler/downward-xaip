from xaip.general import utils

def add_question(path, sas_task):

    if not path:
        return

    reader = open(path)
    lines = reader.readlines()
    reader.close()

    #print("---------------------- Question ---------------------")
    #for v in sas_task.variables.value_names:
        #print(v)
    for line in lines:
        elem = line.replace('\n', '')
        #print(elem)

        #fined matching variable value index in the sas_task
        pair_list = utils.literalVarValueList(sas_task, elem, False)
        assert pair_list and len(pair_list) == 1, "No matching fact: " + line + "\n" + str(sas_task.variables.value_names)
        #print(pair_list[0])
        sas_task.addQuestionElem(pair_list[0])
