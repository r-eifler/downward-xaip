#include "output_handler.h"

#include "../tasks/root_task.h"

#include <fstream>
#include <bitset>

using namespace std;
using namespace goalsubset;

OutputHandler::OutputHandler( string file_name, bool has_relaxations) {
    this->file_name = file_name;
    this->has_relaxations = has_relaxations;
}

void OutputHandler::add_collection(std::string  name, std::vector<std::vector<std::string>> mugs){
    this->iteration_names.push_back(name);
    this->mugs_list.push_back(mugs);
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

    for(uint index = 0; index < this->mugs_list.size(); index++){
        string name = this->iteration_names[index];

        outfile << "{\"name\": \"" << name << "\", \"MUGS\": [\n";
        output_one(outfile, mugs_list[index]);
        outfile << "\n]}";
        if(index < this->mugs_list.size() - 1){
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

    string name = this->iteration_names[0];

    outfile << "{\"MUGS\": [\n";
    output_one(outfile, mugs_list[0]);
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