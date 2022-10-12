#include "goal_subset.h"

#include <cassert>

using namespace std;

namespace goalsubset {

GoalSubset::GoalSubset(){
    goals = {};
}

GoalSubset::GoalSubset(std::bitset<64> goals, int max_num_goals): 
    goals(goals), max_num_goals(max_num_goals) {}

vector<GoalSubset> GoalSubset::strengthen() const{
    vector<GoalSubset> new_subsets;

    for (int i = 0; i < max_num_goals; i++){
        if(!goals[i]){
            std::bitset<64> stronger_subset = goals;
            stronger_subset[i] = 1;
            new_subsets.push_back(GoalSubset(stronger_subset, max_num_goals));
        }
    }

    return new_subsets;
}

vector<GoalSubset> GoalSubset::weaken() const{
    vector<GoalSubset> new_subsets;

    for (int i = 0; i < max_num_goals; i++){
        if(goals[i]){
            std::bitset<64> weaker_subset = goals;
            weaker_subset[i] = 0;
            new_subsets.push_back(GoalSubset(weaker_subset, max_num_goals));
        }
    }

    return new_subsets;
}

vector<GoalSubset> GoalSubset::singelten_subsets() const{
     vector<GoalSubset> singeltons;

    for (int i = 0; i < max_num_goals; i++){
        if(goals[i]){
            std::bitset<64> single_goal = 0;
            single_goal[i] = 1;
            singeltons.push_back(GoalSubset(single_goal, max_num_goals));
        }
    }

    return singeltons;
}

GoalSubset GoalSubset::complement() const{
    std::bitset<64> comple = 0;
     for (int i = 0; i < max_num_goals; i++){
        comple[i] = !goals[i];
    }
    return GoalSubset(comple, max_num_goals);
}

GoalSubset GoalSubset::set_union(GoalSubset set) const{
    assert(this->max_num_goals == set.max_num_goals);

    std::bitset<64> union_set = 0;

    for (int i = 0; i < max_num_goals; i++){
        union_set[i] = this->goals[i] || set.goals[i];
    }

    return GoalSubset(union_set, max_num_goals);
}

GoalSubset GoalSubset::set_intersection(GoalSubset set) const{
    assert(this->max_num_goals == set.max_num_goals);

    std::bitset<64> union_set = 0;

    for (int i = 0; i < max_num_goals; i++){
        union_set[i] = this->goals[i] && set.goals[i];
    }

    return GoalSubset(union_set, max_num_goals);
}

void GoalSubset::print() const {
    cout << goals << endl;
}

}