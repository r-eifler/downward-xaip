#include "policy_confidence_distance_function.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace policy;

namespace policy_distance_function {

PolicyConfidenceDistanceFunction::PolicyConfidenceDistanceFunction(const Options &opts)
    : DistanceFunction(opts), 
    url(opts.get<string>("url")),
    policy_client(PolicyClient(url, false)) {
}

void PolicyConfidenceDistanceFunction::initialize(const shared_ptr<AbstractTask> &task) {
    DistanceFunction::initialize(task);
    policy_client.initialize(task);
}

std::vector<float> PolicyConfidenceDistanceFunction::get_distances(const State &state, std::vector<OperatorID> &op_ids) {

    vector<OperatorID> applicable_operators;
    vector<float> operator_probabilities;

    policy_client.get_operators_prob(state, &applicable_operators, &operator_probabilities);

    if(applicable_operators.size() == 0){
        std::cerr << "get_operators_prob retuens no operators altough there exist applicable operators" << std::endl;
        utils::exit_with(utils::ExitCode::REMOTE_POLICY_ERROR);
    }

    if(applicable_operators.size() != op_ids.size()){
        std::cerr << "get_operators_prob retuens differend number of applicable operators" << std::endl;
        utils::exit_with(utils::ExitCode::REMOTE_POLICY_ERROR);
    }

    float max = 0.0;
    // cout << "==================================" << endl;
    for(size_t i = 0; i < operator_probabilities.size(); i++){
        // cout << operator_probabilities[i] << " ";
        if(operator_probabilities[i] > max){
            max = operator_probabilities[i];
        }
    }
    // cout << endl << "--> max: " << max << endl;
    // cout << "--------------------------------" << endl;
    sum_max_probabilities += max;
   
    int index = int(10 * max);
    max_prob_distribution[index] += 1;
    num_tested_states += 1;

    vector<float> distances;
    for(uint i = 0; i < op_ids.size(); i++){
        for(uint j = 0; j < applicable_operators.size(); j++){
            if (op_ids[i] == applicable_operators[j]){
                distances.push_back(max - operator_probabilities[j]);
                // cout << max - operator_probabilities[j] << " ";
                break;
            }
        }
    }
    // cout << endl;
    // cout << "--------------------------------" << endl;
    assert(distances.size() == op_ids.size());
    return distances;
}

static shared_ptr<DistanceFunction> _parse(OptionParser &parser) {
    parser.document_synopsis("TODO", "TODO");
    parser.add_option<string>(
        "url",
        "url of the policy server",
        "localhost:54321");
    utils::add_log_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<PolicyConfidenceDistanceFunction>(opts);
}

static Plugin<DistanceFunction> _plugin("policy_confidence", _parse);
}
