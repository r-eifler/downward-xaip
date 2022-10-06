#include "goal_subset.h"

using namespace std;

namespace goalsubset {

GoalSubset::GoalSubset(){
    goals = {};
}

GoalSubset::GoalSubset(std::bitset<64> goals): 
    goals(goals) {}

vector<GoalSubset> GoalSubset::strengthen() const{
    vector<GoalSubset> new_subsets;

    for (uint i = 0; i < goals.size(); i++){
        if(!goals[i]){
            std::bitset<64> stronger_subset = goals;
            stronger_subset[i] = 1;
            new_subsets.push_back(stronger_subset);
        }
    }

    return new_subsets;
}

vector<GoalSubset> GoalSubset::weaken() const{
    vector<GoalSubset> new_subsets;

    for (uint i = 0; i < goals.size(); i++){
        if(goals[i]){
            std::bitset<64> weaker_subset = goals;
            weaker_subset[i] = 0;
            new_subsets.push_back(weaker_subset);
        }
    }

    return new_subsets;
}

void GoalSubset::print() const {
    cout << goals << endl;
}

}