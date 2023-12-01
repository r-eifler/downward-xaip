#include "policy_confidence_pruning.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace policy;

namespace policy_pruning_method {

PolicyConfidencePruningMethod::PolicyConfidencePruningMethod(const Options &opts)
    : PruningMethod(opts), 
    url(opts.get<string>("url")),
    project_resources(opts.get<bool>("project_resources")),
    policy_client(PolicyClient(url, project_resources)),
    T(opts.get<double>("threshold")) {
}

void PolicyConfidencePruningMethod::initialize(const shared_ptr<AbstractTask> &task) {
    PruningMethod::initialize(task);
    policy_client.initialize(task);
}

void PolicyConfidencePruningMethod::prune(const State &state, std::vector<OperatorID> & op_ids) {

     if(op_ids.size() == 0){
        return;
    }

    vector<OperatorID> applicable_operators;
    vector<float> operator_probabilities;

    policy_client.get_operators_prob(state, &applicable_operators, &operator_probabilities);

    if(applicable_operators.size() == 0){
        std::cerr << "get_operators_prob retuens no operators altough there exist applicable operators" << std::endl;
        utils::exit_with(utils::ExitCode::REMOTE_POLICY_ERROR);
    }

    float max = 0.0;
    // cout << "--------------------------------" << endl;
    for(size_t i = 0; i < operator_probabilities.size(); i++){
        // cout << operator_probabilities[i] << " ";
        if(operator_probabilities[i] > max){
            max = operator_probabilities[i];
        }
        // cout << "Curr Max: " << max << endl;
    }
    // cout << endl;
    sum_max_probabilities += max;
   
    int index = int(10 * max);
    max_prob_distribution[index] += 1;
    num_tested_states += 1;

    // cout << "Max: " << max << endl;

    vector<OperatorID> remaining_op_ids;
    remaining_op_ids.reserve(op_ids.size());
    for(size_t i = 0; i < operator_probabilities.size(); i++){
        // cout << max - operator_probabilities[i] << endl;
        if(max - operator_probabilities[i] <= T){
            remaining_op_ids.push_back(applicable_operators[i]);
        }
    }
    // cout << "Num operators: " << remaining_op_ids.size() << endl;
    op_ids.swap(remaining_op_ids);
}

static shared_ptr<PruningMethod> _parse(OptionParser &parser) {
    parser.document_synopsis("TODO", "TODO");
    parser.add_option<string>(
        "url",
        "url of the policy server",
        "localhost:54321");
    parser.add_option<double>(
        "threshold",
        "threshold for policy confidence",
        "1",
        Bounds("0", "1"));
    parser.add_option<bool>(
        "project_resources",
        "project to a task without encoded resources before sending",
        "true");
    add_pruning_options_to_parser(parser);

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<PolicyConfidencePruningMethod>(opts);
}

static Plugin<PruningMethod> _plugin("policy_confidence", _parse);
}
