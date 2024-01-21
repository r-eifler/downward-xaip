#include "policy_diverging_actions_distance_function.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace policy;

namespace policy_distance_function {

PolicyDivergingActionsDistanceFunction::PolicyDivergingActionsDistanceFunction(const Options &opts)
    : DistanceFunction(opts), 
    url(opts.get<string>("url")),
    policy_client(PolicyClient(url, project_resources)) {
}

void PolicyDivergingActionsDistanceFunction::initialize(const shared_ptr<AbstractTask> &task) {
    cout << "Initialize PolicyDivergingActionsDistanceFunction ... " << endl;
    DistanceFunction::initialize(task);
    policy_client.initialize(task);

    policy_evaluation_timer.reset();
}

void PolicyDivergingActionsDistanceFunction::notify_initial_state(const State & initial_state) {
    // cout << "notify_initial_state" << endl;
    parent_map.emplace(initial_state.get_id(),initial_state.get_id());
    divergence_count.emplace(initial_state.get_id(),0);

    // TaskProxy task_proxy = TaskProxy(*(this->task));
    // for(size_t i = 0; i < task_proxy.get_variables().size(); i++){
    //     cout << task_proxy.get_variables()[i].get_name() << endl;
    //     for(int j = 0; j < task_proxy.get_variables()[i].get_domain_size(); j++){
    //         cout << task_proxy.get_variables()[i].get_fact(j).get_name() << endl;
    //     }
    // }
}


void PolicyDivergingActionsDistanceFunction::notify_state_transition(const State & parent_state, OperatorID op_id, const State & state){
    // cout << "notify_state_transition: " << parent_state.get_id() << " --  " << op_id << " --> " << state.get_id() << endl;
    TaskProxy task_proxy = TaskProxy(*(this->task));
    // cout << task_proxy.get_operators()[op_id].get_name() << endl;
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

std::vector<float> PolicyDivergingActionsDistanceFunction::get_distances(const State &state, std::vector<OperatorID> &op_ids) {

    // TaskProxy task_proxy = TaskProxy(*(this->task));

    // cout << "-------- prune start ----------" << endl;
    // cout << "State: " << state.get_id() << endl;
    // for(size_t i = 0; i < state.size(); i++){
    //     VariableProxy var_proxy = task_proxy.get_variables()[state[i].get_variable().get_id()];
    //     cout << var_proxy.get_fact(state[i].get_value()).get_name() << endl;
    // }
    // cout << "Num applicable actions: " << op_ids.size() << endl;

    // policy_evaluation_timer.resume();
    OperatorID opId = policy_client.apply(state);
    // policy_evaluation_timer.stop();

    // cout << "Policy action: " << opId << endl;  
    // cout << task_proxy.get_operators()[opId].get_name()  << endl;

    policy_action.emplace(state.get_id(),opId);

    int num_divergence = divergence_count[state.get_id()];

    vector<float> distances;
    for(uint i = 0; i < op_ids.size(); i++){
        if(op_ids[i] == opId){
            distances.push_back(num_divergence);
        }
        else {
            distances.push_back(num_divergence + 1);
        }

    }

    return distances;
}

void PolicyDivergingActionsDistanceFunction::print_statistics() const {
    cout << "Policy evaluation time: " << policy_evaluation_timer << endl;
}

static shared_ptr<DistanceFunction> _parse(OptionParser &parser) {

    parser.document_synopsis("TODO", "TODO");
    parser.add_option<string>(
        "url",
        "url of policy server",
        "localhost:54321");
    utils::add_log_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<PolicyDivergingActionsDistanceFunction>(opts);
}

static Plugin<DistanceFunction> _plugin("policy_action", _parse);
}
