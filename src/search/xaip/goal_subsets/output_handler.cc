#include "output_handler.h"

#include "../tasks/root_task.h"

#include <fstream>
#include <bitset>

using namespace std;
using namespace goalsubset;

OutputHandler::OutputHandler(std::vector<std::string> goal_fact_names, string file_name, bool has_relaxations) {
    this->goal_fact_names = goal_fact_names;
    this->file_name = file_name;
    this->has_relaxations = has_relaxations;
}

void OutputHandler::add_goal_subsets(string  name, GoalSubsets goal_subsets) {
    this->iteration_names.push_back(name);
    this->goal_subsets_list.push_back(goal_subsets);
}


vector<vector<string>> OutputHandler::generate_string(int index) {
    vector<vector<string>> mugs_facts_names;
    int num_goal_facts = goal_fact_names.size();

    auto it = this->goal_subsets_list[index].begin();
    while(it != this->goal_subsets_list[index].end()) {
        GoalSubset gs = *it;
        it++;

        // get names of all facts contained in the OutputHandler
        vector<string> fact_names;
        for (int i = 0; i < num_goal_facts; i++) {
            if (gs.contains(i)) {
                fact_names.push_back(goal_fact_names[i]);
            }
        }
        mugs_facts_names.push_back(fact_names);
    }
    return mugs_facts_names;
}


void OutputHandler:: output_one(ofstream& outfile, std::vector<std::vector<std::string>> facts_names){
    auto it = facts_names.begin();
    while(it != facts_names.end()){
        vector<string> fact_names = *it;

        // print fact names
        outfile << "\t[";
        for(uint i = 0; i < fact_names.size(); i++){
            outfile << "\"" << fact_names[i] << "\"";
            if (i < fact_names.size() - 1){
                outfile << ", ";
            }
        }

        outfile << "]";
        if (next(it) != facts_names.end()){
            outfile << ",\n";
        }
        it++;
    }
}

void OutputHandler::output_relaxations(){
    cout << "---------------- Print MUGS to FILE ----------------" << endl;
    ofstream outfile;
    outfile.open(this->file_name);
    outfile << "{\"relaxations\": [\n";

    for(uint index = 0; index < this->goal_subsets_list.size(); index++){

        vector<vector<string>> mugs_facts_names = this->generate_string(index);
        string name = this->iteration_names[index];

        outfile << "{\"name\": \"" << name << "\", \"MUGS\": [\n";
        output_one(outfile, mugs_facts_names);
        outfile << "\n]}";
        if(index < this->goal_subsets_list.size() - 1){
            outfile << ",\n";
        }
    }

    outfile << "\n]}\n";
    outfile.close();
}

void OutputHandler::output_one(){
    cout << "---------------- Print OutputHandler to FILE ----------------" << endl;
    ofstream outfile;
    outfile.open("mugs.json");

    vector<vector<string>> mugs_facts_names = this->generate_string(0);
    string name = this->iteration_names[0];

    outfile << "{\"OutputHandler\": [\n";
    output_one(outfile, mugs_facts_names);
    outfile << "\n]}";

    outfile.close();
}

void OutputHandler::output() {
   if (this->has_relaxations) {
       output_relaxations();
   }
   else{
       output_one();
   }
}