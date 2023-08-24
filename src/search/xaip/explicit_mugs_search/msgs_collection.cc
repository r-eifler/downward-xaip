#include "msgs_collection.h"

#include "../../tasks/root_task.h"

#include <fstream>
#include <bitset>

using namespace std;
using namespace goalsubset;

MSGSCollection::MSGSCollection() {}

void MSGSCollection::initialize(shared_ptr<AbstractTask> task_) {

    cout << "---------------- INIT MSGS Collection ---------------------- " << endl;
    if(soft_goal_list.size() > 0){
        cout << "--> already initialized" << endl;
        return;
    }

    num_visited_states_since_last_added = 0;

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

    // cout << "#hard goals: " << hard_goal_list.size() << endl;

    //sort goal facts according to there variable id
    std::sort(hard_goal_list.begin(), hard_goal_list.end());

    // cout << "#soft goals: " << soft_goal_list.size() << endl;

    //sort goal facts according to there variable id
    std::sort(soft_goal_list.begin(), soft_goal_list.end());

    soft_goal_fact_names = std::vector<std::string>(soft_goal_list.size());
    for(uint i = 0; i < soft_goal_list.size(); i++){
        FactPair gp = soft_goal_list[i];
        string name = task_proxy.get_variables()[gp.var].get_fact(gp.value).get_name();
        soft_goal_fact_names[i] = name;
        cout << gp.var << " = " << gp.value  << "    -->  id: " << i << " " << name << endl;
    }

    // init with empty set 
    this->add(GoalSubset(soft_goal_list.size()));

    overall_timer.reset();
}

void MSGSCollection::add_and_mimize(GoalSubset subset){
    assert(soft_goal_list.size() == subset.size());
    this->add(subset);
    this->minimize_non_maximal_subsets();
}

void MSGSCollection::add_and_mimize(GoalSubsets subsets) {
    for (GoalSubset gs : subsets){
        add_and_mimize(gs);
    }
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
    GoalSubset hard_goals = GoalSubset(hard_goal_list.size());
    for(size_t i = 0; i < hard_goal_list.size(); i++){
        hard_goals.set(i, reachable_goals.contains(i));
    }
    return hard_goals;
}

vector<FactPair> MSGSCollection::get_goal_facts() {
    return all_goal_list;
}

void MSGSCollection::update_best_state(StateID id, int num_solved_soft_goals) {
    if (num_solved_soft_goals >= max_num_solved_soft_goals){
        best_state = id;
        max_num_solved_soft_goals = num_solved_soft_goals;
    }
}


bool MSGSCollection::prune(const State &state, vector<int> costs, int remaining_cost){

    // costs containes the costs of the facts in all_goal_list (in the same order)
    overall_timer.resume();
    num_visited_states_since_last_added++;

    // cout<< "-------------- CURRENT MSGS ------------------" << endl;
    // this->print_subsets();
    // cout<< "-------------- CURRENT MSGS ------------------" << endl;

    GoalSubset reachable_goals = GoalSubset(all_goal_list.size());
    for(size_t i = 0; i < all_goal_list.size(); i++){
        // in FD the cost bound is strict thus we can test for < instead of <=
        reachable_goals.set(i, costs[i] != -1 && costs[i] < remaining_cost); 
    }

    // cout << "reachable goals: " << endl;
    // reachable_goals.print();

    if(hard_goal_list.size() == 0){
        if(this->contains_superset(reachable_goals)){
            // cout<< "contains superset" << endl;
            num_pruned_states += 1;
            overall_timer.stop();
            return true;
        }
        else{
            GoalSubset satisfied_goals = get_satisfied_all_goals(state);
            update_best_state(state.get_id(), satisfied_goals.count());
            assert(reachable_goals.is_superset_of(satisfied_goals));
            // cout << "satisfied goals: " << endl;
            // satisfied_goals.print();
            if(!contains_superset(satisfied_goals)){
                this->add_and_mimize(satisfied_goals);
                // cout << "Num states since last add new goal subset: " << num_visited_states_since_last_added << endl;
                num_visited_states_since_last_added = 0;
            }
            overall_timer.stop();
            return false;
        }
    }
    else{
        GoalSubset satisfied_hard_goals = get_satisfied_hard_goals(state);
        GoalSubset satisfied_soft_goals = get_satisfied_soft_goals(state);

        GoalSubset reachable_hard_goals = get_reachable_hard_goals(reachable_goals);
        GoalSubset reachable_soft_goals = get_reachable_soft_goals(reachable_goals);

        bool superset_alreday_readched = this->contains_superset(reachable_soft_goals);

        //TODO does it make sense to check whether a superset of soft goals is reachable?
        if(! reachable_hard_goals.all() || superset_alreday_readched){
            // cout<< "contains superset" << endl;
            num_pruned_states += 1;
            return true;
        }
        else{
            update_best_state(state.get_id(), satisfied_soft_goals.count());
            if(satisfied_hard_goals.all() && !contains_superset(satisfied_soft_goals)){
                this->add_and_mimize(satisfied_soft_goals);
                // cout<< "add new goal subset" << endl;
            }
            if(superset_alreday_readched)
                num_pruned_states += 1;
            return superset_alreday_readched;
        }

    }
}

bool MSGSCollection::track(const State &state){

    if(hard_goal_list.size() == 0){
        GoalSubset satisfied_goals = get_satisfied_all_goals(state);
        update_best_state(state.get_id(), satisfied_goals.count());
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
             update_best_state(state.get_id(), satisfied_soft_goals.count());
            this->add_and_mimize(satisfied_soft_goals);
            return true;
        }
        return false;
    }
}

GoalSubsets MSGSCollection::get_mugs() const{
    return this->complement().minimal_hitting_sets();
}

void MSGSCollection::print() const {

    utils::Timer hit_timer;
    GoalSubsets mugs = this->complement().minimal_hitting_sets();
    hit_timer.stop();

    cout << "HIT computation: " << hit_timer << endl;
    cout << "Prunig time: " << overall_timer << endl;
    cout << "#pruned states: " << num_pruned_states << endl;
    // cout << "Final Num states since last add new goal subset: " << num_visited_states_since_last_added << endl;
    cout << "*********************************"  << endl;
    cout << "#hard goals: " << hard_goal_list.size() << endl;
    TaskProxy taskproxy = TaskProxy(*tasks::g_root_task.get());
    for(FactPair g : hard_goal_list){
        cout << "\t" << taskproxy.get_variables()[g.var].get_fact(g.value).get_name() << endl;
    }
    cout << "#soft goals: " << soft_goal_list.size() << endl;
    for(uint i = 0; i < soft_goal_fact_names.size(); i++){
        cout << "\t" << i << ": " << soft_goal_fact_names[i] << endl;
    }

    cout << "*********************************"  << endl;
    cout << "#MSGS: " << this->size() << endl;
    cout << "*********************************"  << endl;
    if(this->size() > 150){
        cout << "Too many msgs to print!" << endl;
    }
    else{
        this->print_subsets();
    }
    cout << "*********************************"  << endl;
    cout << "*********************************"  << endl;
    cout << "#MUGS: " << mugs.size() << endl;
    cout << "*********************************"  << endl;
    if(mugs.size() > 150){
        cout << "Too many mugs to print!" << endl;
    }
    else{
        mugs.print_subsets();
    }
    cout << "*********************************"  << endl;
    if(mugs.size() > 150){
        cout << "Too many mugs to print!" << endl;
    }
    else{
        mugs.print(soft_goal_fact_names);
    }
    cout << "*********************************"  << endl;
}


vector<vector<string>> MSGSCollection::generate_msgs_string() {
    return GoalSubsets::generate_string(soft_goal_fact_names);
}

vector<vector<string>> MSGSCollection::generate_mugs_string() {
    return get_mugs().generate_string(soft_goal_fact_names);
}



