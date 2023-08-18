#ifndef FAST_DOWNWARD_OUTPUT_HANDLER
#define FAST_DOWNWARD_OUTPUT_HANDLER

#include "../task_proxy.h"
#include "goal_subset.h"
#include "goal_subsets.h"
#include "../explicit_mugs_search/msgs_collection.h"
#include <iostream>
#include <string>


class OutputHandler {
    bool has_relaxations;
    std::string file_name;

    std::vector<std::vector<std::vector<std::string>>> mugs_list;
    std::vector<std::string> iteration_names;


    void output_one(std::ofstream& outfile, std::vector<std::vector<std::string>> facts_names);
    void output_relaxations();
    void output_one();

public:
    explicit OutputHandler(std::string file_name = "results.json", bool relaxations = true);
    void add_collection(std::string  name, std::vector<std::vector<std::string>> mugs);
    void output();
};


#endif //FAST_DOWNWARD_OUTPUT_HANDLER
