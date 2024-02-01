#include "potential_goals_heuristic.h"

#include "potential_function.h"
#include "../option_parser.h"
#include <algorithm>

using namespace std;

namespace potentials {
PotentialGoalsHeuristic::PotentialGoalsHeuristic(
    const Options &opts,
    vector<unique_ptr<PotentialFunction>> &&functions)
    : Heuristic(opts){

        this->functions = move(functions);
}
    

int PotentialGoalsHeuristic::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);
    int value = 0;
    for (auto &function : functions) {
        int e = max(0, function->get_value(state));
        if (e == std::numeric_limits<int>::max()){
            return std::numeric_limits<int>::max() - 10;
        }
        value += e;
    }
    return value;
}

std::vector<int> PotentialGoalsHeuristic::get_heuristic_values(const State &ancestor_state, std::vector<FactPair>){

    // cout << "----------------------------" << endl;
    vector<int> result;
    State state = convert_ancestor_state(ancestor_state);
    for (auto &function : functions) {
        int value = max(0,function->get_value(state));
        if (value >= 100000000){
            result.push_back(-1);
        }
        else{
            result.push_back(value);
        }
        // cout << "h= " << value << endl;
    }
    
    return result;
}

}
