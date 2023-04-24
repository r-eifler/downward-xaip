#include "msgs_collection.h"

#include "../../tasks/root_task.h"

#include <fstream>
#include <bitset>

using namespace std;
using namespace goalsubset;

MSGSCollection::MSGSCollection() {}

void MSGSCollection::initialize(shared_ptr<AbstractTask> task_) {

    cout << "---------------- INIT MSGS Collection ---------------------- " << endl;

    assert(!task_);
    task = task_;

    TaskProxy task_proxy = TaskProxy(*task);

    soft_goal_list = vector<FactPair>();
    hard_goal_list = vector<FactPair>();

    if(task_proxy.get_hard_goals().size() == 0){
        cout << "--> all goals are soft goals" << endl;
        for(uint i = 0; i < task_proxy.get_goals().size(); i++){
            soft_goal_list.push_back(task_proxy.get_goals()[i].get_pair());
            all_goal_list.push_back(task_proxy.get_goals()[i].get_pair());
        }
    }
    else{
        for(uint i = 0; i < task_proxy.get_hard_goals().size(); i++){
            hard_goal_list.push_back(task_proxy.get_hard_goals()[i].get_pair());
            all_goal_list.push_back(task_proxy.get_hard_goals()[i].get_pair());
        }
        for(uint i = 0; i < task_proxy.get_soft_goals().size(); i++){
            soft_goal_list.push_back(task_proxy.get_soft_goals()[i].get_pair());
            all_goal_list.push_back(task_proxy.get_soft_goals()[i].get_pair());
        }
    }

    cout << "#soft goals: " << soft_goal_list.size() << endl;

    //sort goal facts according to there variable id
    std::sort(soft_goal_list.begin(), soft_goal_list.end());

    soft_goal_fact_names = std::vector<std::string>();
    for(uint i = 0; i < soft_goal_list.size(); i++){
        FactPair gp = soft_goal_list[i];
        string name = task_proxy.get_variables()[gp.var].get_fact(gp.value).get_name();
        soft_goal_fact_names.push_back(name);
        cout << gp.var << " = " << gp.value  << "    -->  " << name << endl;
    }
}

void MSGSCollection::add_and_mimize(GoalSubset subset){
    this->add(subset);
    this->minimize_non_maximal_subsets();
}

bool MSGSCollection::contains_superset(GoalSubset subset){
    for(GoalSubset s : subsets){
        if(s.is_superset_of(subset)){
            return true;
        }
    }
    return false;
}

bool MSGSCollection::contains_strict_superset(GoalSubset subset){
    for(GoalSubset s : subsets){
        if(s.is_strict_superset_of(subset)){
            return true;
        }
    }
    return false;
}


GoalSubset MSGSCollection::get_satisfied_soft_goals(const State &state){
    GoalSubset soft_goals = GoalSubset(soft_goal_list.size());
    for(size_t i = 0; i < soft_goal_list.size(); i++){
        FactPair g = soft_goal_list[i];
        if(state[g.var].get_value() == g.value){
            soft_goals.add(i);
        }
    }
    return soft_goals;
}


GoalSubset MSGSCollection::get_satisfied_hard_goals(const State &state){
    GoalSubset hard_goals = GoalSubset(hard_goal_list.size());
    for(size_t i = 0; i < hard_goal_list.size(); i++){
        FactPair g = hard_goal_list[i];
        if(state[g.var].get_value() == g.value){
            hard_goals.add(i);
        }
    }
    return hard_goals;
}


GoalSubset MSGSCollection::get_satisfied_all_goals(const State &state){
    GoalSubset goals = GoalSubset(all_goal_list.size());
   for(size_t i = 0; i < all_goal_list.size(); i++){
        FactPair g = all_goal_list[i];
        if(state[g.var].get_value() == g.value){
            goals.add(i);
        }
    }
    return goals;
}

GoalSubset MSGSCollection::get_reachable_soft_goals(GoalSubset reachable_goals){
    GoalSubset soft_goals = GoalSubset(soft_goal_list.size());
    for(size_t i = 0; i < soft_goal_list.size(); i++){
        soft_goals.set(i, reachable_goals.contains(hard_goal_list.size() + i));
    }
    return soft_goals;
}

GoalSubset MSGSCollection::get_reachable_hard_goals(GoalSubset reachable_goals){
    GoalSubset hard_goals = GoalSubset(soft_goal_list.size());
    for(size_t i = 0; i < hard_goal_list.size(); i++){
        hard_goals.set(i, reachable_goals.contains(i));
    }
    return hard_goals;
}

vector<FactPair> MSGSCollection::get_goal_facts() {
    return all_goal_list;
}


bool MSGSCollection::prune(const State &state, vector<int> costs, int remaining_cost){

    // costs containes the costs of the facts in all_goal_list (in the same order)

    GoalSubset reachable_goals = GoalSubset(all_goal_list.size());
    for(size_t i = 0; i < all_goal_list.size(); i++){
        reachable_goals.set(i,costs[i] != -1 && costs[i] <= remaining_cost);
    }

    // cout << "reachable goals: " << endl;
    // reachable_goals.print();


    if(hard_goal_list.size() == 0){
        if(this->contains_superset(reachable_goals)){
            // cout<< "contains superset" << endl;
            return true;
        }
        else{
            GoalSubset satisfied_goals = get_satisfied_all_goals(state);
            // cout << "satisfied goals: " << endl;
            // satisfied_goals.print();
            if(!contains_superset(satisfied_goals)){
                this->add_and_mimize(satisfied_goals);
                // cout<< "add new goal subset" << endl;
            }
            return false;
        }
    }
    else{
        GoalSubset satisfied_hard_goals = get_satisfied_hard_goals(state);
        GoalSubset satisfied_soft_goals = get_satisfied_soft_goals(state);

        GoalSubset reachable_hard_goals = get_reachable_soft_goals(reachable_goals);
        GoalSubset reachable_soft_goals = get_reachable_hard_goals(reachable_goals);

        bool superset_alreday_readched = this->contains_superset(reachable_soft_goals);

        //TODO does it make sense to check whether a superset of soft goals is reachable?
        if(! reachable_hard_goals.all() || superset_alreday_readched){
            // cout<< "contains superset" << endl;
            return true;
        }
        else{
            if(satisfied_hard_goals.all() && !contains_superset(satisfied_soft_goals)){
                this->add_and_mimize(satisfied_soft_goals);
                // cout<< "add new goal subset" << endl;
            }
            return superset_alreday_readched;
        }

    }
}

bool MSGSCollection::track(const State &state){

    if(hard_goal_list.size() == 0){
        GoalSubset satisfied_goals = get_satisfied_all_goals(state);
        // cout << "satisfied goals: " << endl;
        // satisfied_goals.print();
        if(!contains_superset(satisfied_goals)){
            this->add_and_mimize(satisfied_goals);
            // cout<< "add new goal subset" << endl;
            return true;
        }
        return false;
    }
    else{
        GoalSubset satisfied_hard_goals = get_satisfied_hard_goals(state);
        GoalSubset satisfied_soft_goals = get_satisfied_soft_goals(state);

        if(satisfied_hard_goals.all() && !contains_superset(satisfied_soft_goals)){
            this->add_and_mimize(satisfied_soft_goals);
            // cout<< "add new goal subset" << endl;
            return true;
        }
        return false;
    }
}

void MSGSCollection::print() const {
    GoalSubsets mugs = this->complement().minimal_hitting_sets();

    cout << "*********************************"  << endl;
    cout << "#hard goals: " << hard_goal_list.size() << endl;
    TaskProxy taskproxy = TaskProxy(*tasks::g_root_task.get());
    for(FactPair g : hard_goal_list){
        cout << "\t" << taskproxy.get_variables()[g.var].get_fact(g.value).get_name() << endl;
    }
    cout << "#soft goals: " << soft_goal_list.size() << endl;
    for(FactPair g : soft_goal_list){
        cout << "\t" << taskproxy.get_variables()[g.var].get_fact(g.value).get_name() << endl;
    }
    cout << "*********************************"  << endl;
    cout << "#MSGS: " << this->size() << endl;
    cout << "*********************************"  << endl;
    this->print_subsets();
    cout << "*********************************"  << endl;
    cout << "*********************************"  << endl;
    cout << "#MUGS: " << mugs.size() << endl;
    cout << "*********************************"  << endl;
    mugs.print_subsets();
    cout << "*********************************"  << endl;
    mugs.print(soft_goal_fact_names);
    cout << "*********************************"  << endl;
}



