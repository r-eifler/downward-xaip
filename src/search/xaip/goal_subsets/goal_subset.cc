#include "goal_subset.h"

#include <cassert>

using namespace std;

namespace goalsubset {

GoalSubset::GoalSubset(){
    goals = boost::dynamic_bitset<>(0,0);
}

GoalSubset::GoalSubset(size_t max_num_goals){
    goals = boost::dynamic_bitset<>(max_num_goals,0);
}

GoalSubset::GoalSubset(size_t max_num_goals, size_t index){
    goals = boost::dynamic_bitset<>(max_num_goals, 0);
    goals[index] = true;
}

GoalSubset::GoalSubset(boost::dynamic_bitset<> goals): 
    goals(goals) {}


GoalSubset GoalSubset::clone() const{
    return GoalSubset(this->goals);
}

vector<GoalSubset> GoalSubset::strengthen() const{
    vector<GoalSubset> new_subsets;

    for (long unsigned int i = 0; i < goals.size(); i++){
        if(!goals[i]){
            boost::dynamic_bitset<> stronger_subset = goals;
            stronger_subset[i] = 1;
            new_subsets.push_back(GoalSubset(stronger_subset));
        }
    }

    return new_subsets;
}

vector<GoalSubset> GoalSubset::weaken() const{
    vector<GoalSubset> new_subsets;

    for (long unsigned int i = 0; i < goals.size(); i++){
        if(goals[i]){
            boost::dynamic_bitset<> weaker_subset = goals;
            weaker_subset[i] = 0;
            new_subsets.push_back(GoalSubset(weaker_subset));
        }
    }

    return new_subsets;
}

vector<GoalSubset> GoalSubset::singelten_subsets() const{
     vector<GoalSubset> singeltons;

    for (long unsigned int i = 0; i < goals.size(); i++){
        if(goals[i]){
            boost::dynamic_bitset<> single_goal =  boost::dynamic_bitset<>(goals.size(), 0);
            single_goal[i] = 1;
            singeltons.push_back(GoalSubset(single_goal));
        }
    }

    return singeltons;
}

GoalSubset GoalSubset::complement() const{
    boost::dynamic_bitset<> comple = boost::dynamic_bitset<>(goals.size(), 0);
     for (long unsigned int i = 0; i < goals.size(); i++){
        comple[i] = !goals[i];
    }
    return GoalSubset(comple);
}

GoalSubset GoalSubset::set_union(GoalSubset set) const{
    assert(this->size() == set.size());

    boost::dynamic_bitset<> union_set = boost::dynamic_bitset<>(goals.size(), 0);

    for (long unsigned int i = 0; i < goals.size(); i++){
        union_set[i] = this->goals[i] || set.goals[i];
    }

    return GoalSubset(union_set);
}

GoalSubset GoalSubset::set_intersection(GoalSubset set) const{
    assert(this->size() == set.size());

   boost::dynamic_bitset<> union_set =  boost::dynamic_bitset<>(goals.size(), 0);

    for (long unsigned int i = 0; i < goals.size(); i++){
        union_set[i] = this->goals[i] && set.goals[i];
    }

    return GoalSubset(union_set);
}

void GoalSubset::print() const {
    cout << goals << endl;
}

}