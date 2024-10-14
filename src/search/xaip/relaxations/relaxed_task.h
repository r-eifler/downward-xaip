#include <string>
#include <iostream>
#include <set>

#ifndef FAST_DOWNWARD_RELAXED_TASK_H
#define FAST_DOWNWARD_RELAXED_TASK_H

#include "../../task_proxy.h"
#include "../goal_subsets/goal_subsets.h"
#include "../explicit_mugs_search/msgs_collection.h"
#include "frontier_elem.h"

class RelaxedTask {

private:
    int id;
    std::string name;
    std::vector<FactPair> init;
    std::vector<ApplicableActionDefinition> applicable_actions;
    std::vector<RelaxedTask*> lower_cover;
    std::vector<RelaxedTask*> upper_cover;
    std::unordered_set<FrontierElem, HashFrontierElem> frontier;
    MSGSCollection msgs_collection;
    bool solvable = false;
    int expanded_states = 0;
    uint num_accessed_frontier = 0;
public:
    RelaxedTask(std::shared_ptr<AbstractTask> task, int id, std::string name, 
    std::vector<FactPair> init, std::vector<ApplicableActionDefinition> appla);

    int get_id(){return id;}
    std::string get_name(){return name;}
    std::vector<FactPair> get_init(){return init;}

    std::vector<RelaxedTask*> get_lower_cover(){return lower_cover;}
    std::set<RelaxedTask*> get_lower_bound();
    void add_to_lover_cover(RelaxedTask* task){lower_cover.push_back(task);}

    std::vector<RelaxedTask*> get_upper_cover(){return upper_cover;}
    void add_to_upper_cover(RelaxedTask* task){upper_cover.push_back(task);}

    std::unordered_set<FrontierElem, HashFrontierElem> get_frontier(){
        num_accessed_frontier++; 
        return frontier;
    }
    uint get_frontier_size(){return frontier.size();}
    void add_to_frontier(FrontierElem elem){frontier.insert(elem);}

    void set_num_expanded_states(int num) {this->expanded_states = num;}
    int get_num_expanded_states() {return this->expanded_states;}

    MSGSCollection get_msgs() const {return msgs_collection;}
    void add_msgs(MSGSCollection msgs_collection){this->msgs_collection.add_and_mimize(msgs_collection);}

    std::vector<std::vector<std::string>> get_mugs_string();
    std::vector<std::vector<std::string>> get_msgs_string();

    void set_solvable(bool s){solvable = s;}
    bool get_solvable(){return solvable;}

    bool applicable(const OperatorProxy &op);
    void propagate_solvable(MSGSCollection goal_subsets);
    void propagate_solvable();
    void clear();

    void print();

};


#endif //FAST_DOWNWARD_RELAXED_TASK_H
