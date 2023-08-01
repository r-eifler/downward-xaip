#include "relaxed_task.h"

using namespace std;

RelaxedTask::RelaxedTask(int id, string name, std::vector<FactPair> init, std::string limit_type,
                         std::vector<FactPair> limits):
        id(id), name(name), init(init), limit_type(limit_type), limits(limits){
    cout << "Init relaxation: " << name << endl;
}

bool RelaxedTask::sat_limits(const State &state){
    if (limit_type == "OR"){
        for (FactPair fp : limits){
            if (state[fp.var].get_value() == fp.value){
                return true;
            }
        }
        return false;
    }

    if (limit_type == "AND"){
        for (FactPair fp : limits){
            if (state[fp.var].get_value() != fp.value){
                return false;
            }
        }
        return true;
    }

    return true;
}

void RelaxedTask::propagate_solvable(MSGSCollection goal_subsets){
    // TODO remove overhead from calling the same task multiple times
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
    msgs_collection.print();
    cout << "-----------------------------------------------------" << endl;
}