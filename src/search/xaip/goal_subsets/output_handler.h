#ifndef FAST_DOWNWARD_OUTPUT_HANDLER
#define FAST_DOWNWARD_OUTPUT_HANDLER

#include "../task_proxy.h"
#include "goal_subset.h"
#include "goal_subsets.h"
#include <iostream>
#include <string>


class OutputHandler {
    bool has_relaxations;
    std::string file_name;

    std::vector<GoalSubsets> goal_subsets_list;
    std::vector<std::string> goal_fact_names;
    std::vector<std::string> iteration_names;


    std::vector<std::vector<std::string>> generate_string(int index);
    void output_one(std::ofstream& outfile, std::vector<std::vector<std::string>> facts_names);
    void output_relaxations();
    void output_one();

public:
    explicit OutputHandler(std::vector<std::string> goal_fact_names, std::string file_name = "results.json", bool relaxations = true);
    void add_goal_subsets(std::string  name, GoalSubsets goal_subsets);
    void output();
};


#endif //FAST_DOWNWARD_OUTPUT_HANDLER
