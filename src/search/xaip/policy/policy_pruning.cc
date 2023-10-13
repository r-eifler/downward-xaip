#include "policy_pruning.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace policy;

namespace policy_pruning_method {

PolicyPruningMethod::PolicyPruningMethod(const Options &opts)
    : PruningMethod(opts), 
    port(opts.get<int>("port")),
    project_resources(opts.get<bool>("project_resources")),
    policy_client(PolicyClient(port, project_resources)) {
}

void PolicyPruningMethod::initialize(const shared_ptr<AbstractTask> &task) {
    PruningMethod::initialize(task);
    policy_client.initialize(task);
}

void PolicyPruningMethod::prune(const State &state, std::vector<OperatorID> & op_ids) {
    vector<double> policy_values = policy_client.get_value(state, op_ids);

    double max = 0.0;
    for(size_t i = 0; i < policy_values.size(); i++){
        if(policy_values[i] > max){
            max = policy_values[i];
        }
        // cout << "Curr Max: " << max << endl;
    }

    // cout << "Max: " << max << endl;

    // Now check which applicable operators are in the stubborn set.
    vector<OperatorID> remaining_op_ids;
    remaining_op_ids.reserve(op_ids.size());
    for(size_t i = 0; i < policy_values.size(); i++){
        if(max - policy_values[i] <= T){
            remaining_op_ids.push_back(op_ids[i]);
        }
    }
    // cout << "remaining ops: " << remaining_op_ids.size() << endl;
    op_ids.swap(remaining_op_ids);
}

static shared_ptr<PruningMethod> _parse(OptionParser &parser) {
    parser.document_synopsis("TODO", "TODO");
    parser.add_option<int>(
        "port",
        "port for policy server",
        "8888",
        Bounds("0", "infinity"));
    parser.add_option<bool>(
        "project_resources",
        "project to a task without encoded resources before sending",
        "true");
    add_pruning_options_to_parser(parser);

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<PolicyPruningMethod>(opts);
}

static Plugin<PruningMethod> _plugin("policy", _parse);
}
