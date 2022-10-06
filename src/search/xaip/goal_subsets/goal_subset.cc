#include "goal_subset.h"

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

void GoalSubset::print() const {
    cout << goals << endl;
}

}