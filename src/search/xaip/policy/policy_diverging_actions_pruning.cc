#include "policy_diverging_actions_pruning.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace policy;

namespace policy_pruning_method {

PolicyDivergingActionsPruningMethod::PolicyDivergingActionsPruningMethod(const Options &opts)
    : PruningMethod(opts), 
    url(opts.get<string>("url")),
    project_resources(opts.get<bool>("project_resources")),
    policy_client(PolicyClient(url, project_resources)),
    max_diverging_actions(opts.get<int>("max_diverging_actions")) {
}

void PolicyDivergingActionsPruningMethod::initialize(const shared_ptr<AbstractTask> &task) {
    PruningMethod::initialize(task);
    policy_client.initialize(task);

    policy_evaluation_timer.reset();
}

void PolicyDivergingActionsPruningMethod::notify_initial_state(const State & initial_state) {
    // cout << "notify_initial_state" << endl;
    parent_map.emplace(initial_state.get_id(),initial_state.get_id());
    divergence_count.emplace(initial_state.get_id(),0);
}


void PolicyDivergingActionsPruningMethod::notify_state_transition(const State & parent_state, OperatorID op_id, const State & state){
    // cout << "notify_state_transition: " << parent_state.get_id() << " --  " << op_id << " --> " << state.get_id() << endl;
    int new_divergence_count = divergence_count.at(parent_state.get_id()) + (op_id != policy_action.at(parent_state.get_id()));
    if(parent_map.find(state.get_id()) != parent_map.end()){
        //check if the new path has more diverging actions then do not update
        if(new_divergence_count >= divergence_count[state.get_id()]){
            return;
        }
    }

    parent_map.emplace(state.get_id(),parent_state.get_id());
    divergence_count.emplace(state.get_id(), new_divergence_count);
}

void PolicyDivergingActionsPruningMethod::prune(const State &state, std::vector<OperatorID> & op_ids) {

    // cout << "-------- prune start ----------" << endl;
    // cout << "State: " << state.get_id() << endl;
    // cout << "Num applicable actions: " << op_ids.size() << endl;
        if(op_ids.size() == 0){
            // cout << "-------- prune end ----------" << endl;
            return;
    }

    policy_evaluation_timer.resume();
    vector<double> policy_values; // = policy_client.get_value(state, op_ids);
    policy_evaluation_timer.stop();
    // cout << policy_values << endl;

    double max_confidence = 0.0;
    double max_action_id = -1;

    for(size_t i = 0; i < policy_values.size(); i++){
        if(policy_values[i] > max_confidence){
            max_confidence = policy_values[i];
            max_action_id = i;
        }
    }

    // cout << "Policy action: " << op_ids[max_action_id] << endl;

    policy_action.emplace(state.get_id(),op_ids[max_action_id]);

    int num_divergence = divergence_count[state.get_id()];
    // cout << "Divergence count: " << num_divergence << endl;
    if(num_divergence  < max_diverging_actions){
        // cout << "use all ops" << endl;
        // cout << "-------- prune end ----------" << endl;
        return;
    }

    vector<OperatorID> remaining_op_ids;
    remaining_op_ids.push_back(op_ids[max_action_id]);

    // cout << "only use policy ops" << endl;
    op_ids.swap(remaining_op_ids);
    // cout << "-------- prune end ----------" << endl;
}

void PolicyDivergingActionsPruningMethod::print_statistics() const {
    cout << "Policy evaluation time: " << policy_evaluation_timer << endl;
}

static shared_ptr<PruningMethod> _parse(OptionParser &parser) {
    parser.document_synopsis("TODO", "TODO");
    parser.add_option<int>(
        "url",
        "url of policy server",
        "localhost:8888");
    parser.add_option<int>(
        "max_diverging_actions",
        "maximal number of diverging actions",
        "1",
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

    return make_shared<PolicyDivergingActionsPruningMethod>(opts);
}

static Plugin<PruningMethod> _plugin("policy_action", _parse);
}
