#include "relaxed_task.h"

using namespace std;

RelaxedTask::RelaxedTask(std::shared_ptr<AbstractTask> task, int id, string name, 
    std::vector<FactPair> init, std::vector<ApplicableActionDefinition> appla):
    id(id), name(name), init(init), applicable_actions(appla){

    msgs_collection.initialize(task);

    cout << "Init relaxation: " << name << endl;
}

void RelaxedTask::clear(){
    if(num_accessed_frontier >= upper_cover.size()){
        cout << "------------- CLEAR ------------------" << endl;
        this->frontier.clear();
        this->msgs_collection.clear();
    }
}

bool RelaxedTask::applicable(const OperatorProxy &op){
    for (ApplicableActionDefinition ap : this->applicable_actions){
        string name = op.get_name();
        char* char_array = new char[name.length() + 1]; 
        strcpy(char_array, name.c_str()); 
        char * p;
        p = strtok(char_array, " "); 
        if(p != ap.name){
            continue;
        }
        uint index = 0;
        p = strtok(NULL, " ");
        while (p != NULL) {
            // cout << "Next " << p << endl;
            // if(index < ap.params.size()){
            //     cout << "Param: " + ap.params[index] << endl;
            // }
            if(index < ap.params.size() && ap.params[index] != "*"){
                if(ap.params[index] != p){
                    return true;
                }
            }
            if(index == ap.param_id){
                uint x;
                sscanf(p+5, "%u", &x);
                // cout << op.get_name() << endl;
                // cout << "Time: " << x << endl;
                // cout << "lower: " << ap.lower_bound << "   upper: " << ap.upper_bound << endl;
                if (x < ap.lower_bound || x > ap.upper_bound){
                    // cout << "-----------------------------------" << endl;
                    // cout << op.get_name() << endl;
                    // cout << "Time: " << x << endl;
                    // cout << "lower: " << ap.lower_bound << "   upper: " << ap.upper_bound << endl;
                    // cout << "---> not in bound " << endl;
                    return false;
                }
                else{
                    // cout << "-----------------------------------" << endl;
                    // cout << op.get_name() << endl;
                    // cout << "Time: " << x << endl;
                    // cout << "lower: " << ap.lower_bound << "   upper: " << ap.upper_bound << endl;
                }
            }
            index++;
            p = strtok(NULL, " ");
        }
        return true;
    }
    return true;
}

void RelaxedTask::propagate_solvable(){
    if (!solvable){
        return;
    }
    cout << "solved Iteration: " << this->get_name() << endl;
    solvable = true;
    // mugs should be automatically empty
    for (RelaxedTask* rtc : upper_cover){
        rtc->propagate_solvable(msgs_collection);
    }
}

void RelaxedTask::propagate_solvable(MSGSCollection goal_subsets){
    if (solvable){
        return;
    }
    cout << "solved Iteration: " << this->get_name() << endl;
    solvable = true;
    msgs_collection = goal_subsets;
    // mugs should be automatically empty
    for (RelaxedTask* rtc : upper_cover){
        rtc->propagate_solvable(goal_subsets);
    }
}

std::set<RelaxedTask*> RelaxedTask::get_lower_bound(){
    set<RelaxedTask*> tasks;
    if (lower_cover.size() == 0){
        return tasks;
    }
    if (lower_cover.size() == 1){
        tasks.insert(lower_cover[0]);
        return tasks;
    }
    if (lower_cover.size() > 1){
        tasks.insert(lower_cover[0]->lower_cover.begin(), lower_cover[0]->lower_cover.end());
        for (RelaxedTask* rtc : lower_cover){
            vector<RelaxedTask*> intersect;
            set_intersection(tasks.begin(), tasks.end(), rtc->lower_cover.begin(), rtc->lower_cover.end(),
                             std::inserter(intersect, intersect.begin()));
            tasks.clear();
            tasks.insert(intersect.begin(), intersect.end());
        }
        return tasks;
    }

    return tasks;
}

void RelaxedTask::print() {
    cout << "-----------------------------------------------------" << endl;
    cout << "RESULTS: " << this->name << endl;
    cout << "Relaxed Task Expanded States: " << this->expanded_states << endl;
    msgs_collection.print();
    cout << "-----------------------------------------------------" << endl;
}


std::vector<std::vector<std::string>> RelaxedTask::get_mugs_string(){
    return msgs_collection.generate_mugs_string();
}