#include "policy_pruning.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace policy_pruning_method {

PolicyPruningMethod::PolicyPruningMethod(const Options &opts)
    : PruningMethod(opts), 
    radius(opts.get<double>("radius")),
    distance_function(opts.get<shared_ptr<DistanceFunction>>("distance_function")){
}

void PolicyPruningMethod::initialize(const shared_ptr<AbstractTask> &task) {
    cout << "Initialize PolicyPruningMethod ... " << endl;
    PruningMethod::initialize(task);
}

void PolicyPruningMethod::notify_initial_state(const State & initial_state) {
    distance_function->notify_initial_state(initial_state);
}


void PolicyPruningMethod::notify_state_transition(const State & parent_state, OperatorID op_id, const State & state){
    distance_function->notify_state_transition(parent_state,op_id,state);
}

void PolicyPruningMethod::prune(const State &state, std::vector<OperatorID> & op_ids) {
    if(op_ids.size() == 0){
        return;
    }

    vector<float> distances = distance_function->get_distances(state, op_ids);

    vector<OperatorID> remaining_op_ids;

    for(uint i = 0; i < distances.size(); i++){
        if (distances[i] <= radius){
            remaining_op_ids.push_back(op_ids[i]);
        }
        
    }
    // cout << "only use policy ops" << endl;
    op_ids.swap(remaining_op_ids);
    // cout << "-------- prune end ----------" << endl;
}


std::vector<pair<float,OperatorID>> PolicyPruningMethod::postpone(const State &state, std::vector<OperatorID> & op_ids){
    std::vector<pair<float,OperatorID>> postponed_op_ids;

    if(op_ids.size() == 0){
        return postponed_op_ids;
    }

    vector<float> distances = distance_function->get_distances(state, op_ids);
    // cout << "***************************************" << endl;
    // cout << "radius: " << radius << endl;
    // cout << "#ops: " << op_ids.size() << endl;
    // cout << "#dis: " << distances.size() << endl;

    vector<OperatorID> remaining_op_ids;

    for(uint i = 0; i < distances.size(); i++){
        // cout  << distances[i] << " ";
        if (distances[i] <= radius){
            remaining_op_ids.push_back(op_ids[i]);
        }
        else{
            postponed_op_ids.push_back(make_pair(distances[i], op_ids[i]));
        }
    }
    // cout << endl;

    // cout << "only use policy ops" << endl;
    
    // cout << "#ops remaining: " << remaining_op_ids.size() << endl;
    // cout << "#ops postponed: " << postponed_op_ids.size() << endl;
    op_ids.swap(remaining_op_ids);
    // cout << "#ops op_ids: " << op_ids.size() << endl;
    // cout << "-------- prune end ----------" << endl;
    return postponed_op_ids;
}

void PolicyPruningMethod::print_statistics() const {
    
}

static shared_ptr<PolicyPruningMethod> _parse(OptionParser &parser) {
    parser.document_synopsis("TODO", "TODO");
    parser.add_option<double>(
        "radius",
        "TODO",
        "0");
    parser.add_option<shared_ptr<DistanceFunction>>("distance_function", "TODO");
    // add_pruning_options_to_parser(parser);
    utils::add_log_options_to_parser(parser);

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<PolicyPruningMethod>(opts);
}

static PluginTypePlugin<PolicyPruningMethod> _type_plugin(
    "PolicyPruningMethod",
    "Prune or postpone operators.");

static Plugin<PolicyPruningMethod> _plugin("policy_pruning", _parse);
}
