#include "goal_subsets.h"

#include "../../tasks/root_task.h"

#include <fstream>
#include <bitset>

using namespace std;
using namespace goalsubset;

GoalSubsets::GoalSubsets() {}

GoalSubsets::GoalSubsets(GoalSubsetHashSet subsets): subsets(subsets) {}

void GoalSubsets::add(GoalSubset subset) {
    this->subsets.insert(subset);
}

bool GoalSubsets::contains(goalsubset::GoalSubset subset){
    return subsets.find(subset) != subsets.end();
}

void GoalSubsets::add(GoalSubsetHashSet subsets) {
    for(GoalSubset set : subsets){
        this->subsets.insert(set);
    }
}


vector<vector<string>> GoalSubsets::generate_string(vector<string> goal_facts_names) const{
    vector<vector<string>> facts_names;
    int num_goal_facts = goal_facts_names.size();

    auto it = this->subsets.begin();
    while(it != this->subsets.end()) {
        GoalSubset gs = *it;
        it++;

        // get names of all facts contained in the GoalSubsets
        vector<string> fact_names;
        for (int i = 0; i < num_goal_facts; i++) {
            if (gs.contains(i)) {
                fact_names.push_back(goal_facts_names[i]);
            }
        }
        facts_names.push_back(fact_names);
    }
    return facts_names;
}


string GoalSubsets::to_json(vector<vector<string>> facts_names){
    string res = "";

    auto it = facts_names.begin();
    while(it != facts_names.end()){
        vector<string> fact_names = *it;

        // print fact names
        res += "\t[";
        for(uint i = 0; i < fact_names.size(); i++){
            res += "\"" + fact_names[i] + "\"";
            if (i < fact_names.size() - 1){
                res += ", ";
            }
        }

        res += "]";
        if (next(it) != facts_names.end()){
            res += ",\n";
        }
        it++;
    }

    return res;
}

void GoalSubsets::print_subsets() const{
    for(GoalSubset set : subsets){
        set.print();
    }
}

void GoalSubsets::print_subsets(vector<vector<string>> facts_names) const{
    auto it = facts_names.begin();
    while(it != facts_names.end()){
        vector<string> fact_names = *it;

        // print fact names
        for(uint i = 0; i < fact_names.size(); i++){
            cout << fact_names[i];
            if (i < fact_names.size() - 1){
                cout << "|";
            }
        }

        cout << endl;
        it++;
    }
}


void GoalSubsets::to_file(vector<string> goal_facts_names, string filename){
    cout << "---------------- Print GoalSubsets to FILE ----------------" << endl;
    ofstream outfile;
    outfile.open(filename);

    vector<vector<string>> mugs_facts_names = this->generate_string(goal_facts_names);

    outfile << "{\"GoalSubsets\": [\n";
    outfile << to_json(mugs_facts_names);
    outfile << "\n]}";

    outfile.close();
}

void GoalSubsets::print(vector<string> goal_facts_names) const{

    vector<vector<string>> mugs_facts_names = this->generate_string(goal_facts_names);

    print_subsets(mugs_facts_names);
}


void GoalSubsets::minimize_non_minimal_subsets(){
    GoalSubsetHashSet minimized;

    for(GoalSubset add_set : subsets){
        bool add_to_minimized = true;
        auto it = minimized.begin();

        while(it != minimized.end()){
            if(add_set.isSubsetOf(*it)){
                it = minimized.erase(it);
                continue;
            }

            if(add_set.isSupersetOf(*it)){
                add_to_minimized = false;
            }

            it++;
        }

        if(add_to_minimized){
            minimized.insert(add_set);
        }

    }
    this->subsets = minimized;
}

GoalSubsets GoalSubsets::complement() const{
    GoalSubsetHashSet complements;

    for(GoalSubset set : subsets){
        complements.insert(set.complement());
    }

    return GoalSubsets(complements);
}

GoalSubsets GoalSubsets::minus(GoalSubsets sets) const{
    GoalSubsets res_set = GoalSubsets();

    for(GoalSubset s1 : subsets){
        if(!sets.contains(s1))
            res_set.add(s1);
    }

    return res_set;
}

GoalSubsets GoalSubsets::cross_product(GoalSubsets sets) const{

    GoalSubsets product_set = GoalSubsets();

    for(GoalSubset s1 : subsets){
        for(GoalSubset s2 : sets.subsets){
            product_set.add(s1.set_union(s2));
        }
    }

    return product_set;
}


GoalSubsets GoalSubsets::minimal_hitting_sets(){

    GoalSubsets hitting_set = GoalSubsets();

    auto it = subsets.begin();

    for(GoalSubset sing : it->singelten_subsets()){
        hitting_set.add(sing);
    }

    it++;

    while(it != subsets.end()){
        GoalSubset set = *it;
        
        GoalSubsets singeltons = GoalSubsets();
        for(GoalSubset sing : set.singelten_subsets()){
            singeltons.add(sing);
        }

        hitting_set = hitting_set.cross_product(singeltons);
        hitting_set.minimize_non_minimal_subsets();
        it++;
    } 
    return hitting_set;
}
