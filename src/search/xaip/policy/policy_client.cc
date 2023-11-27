#include "policy_client.h"

#include "../option_parser.h"
#include "../plugin.h"

#include <boost/algorithm/string/join.hpp>
#include <string>

using namespace std;

namespace policy {

RemotePolicyError::RemotePolicyError(std::string msg) : msg(std::move(msg)) {}

void RemotePolicyError::print() const {
    std::cerr << "Remote Policy Error: " << msg << std::endl;
}

    PolicyClient::PolicyClient(const string url, bool project_resources): 
    url(url), project_resources(project_resources){}

    void PolicyClient::initialize(const shared_ptr<AbstractTask> &task_) {
        assert(!task);
        task = task_;
    }


void PolicyClient::establish_connection(const std::string &url) {
    utils::g_log << "Establishing connection to remote policy at " << url << std::endl;
    pheromone_policy = phrmPolicyConnect(url.c_str());
    if (!pheromone_policy) {
        throw RemotePolicyError("Cannot connect to " + url);
    }
    utils::g_log << "Connection to " << url << " established" << std::endl;
    g_default_policy = std::make_shared<PolicyClient>(url, false);
}

std::shared_ptr<PolicyClient> PolicyClient::get_global_default_policy() {
    if (!pheromone_policy) {
        throw RemotePolicyError("Global default policy not available, no connection established");
    }
    assert(g_default_policy);
    return g_default_policy;
}

std::string PolicyClient::input_fdr() {
    if (!connection_established()) {
        throw RemotePolicyError("No connection to remote policy established.\n"
                                "Make sure your FD call starts with --remote-policy <url>.");
    }
    char *fdr = phrmPolicyFDRTaskFD(pheromone_policy);
    if (!fdr) {
        throw RemotePolicyError("Cannot obtain FDR task");
    }
    std::string out(fdr);
    free(fdr);
    return out;
}

OperatorID PolicyClient::static_apply(const State &state_in) {
    if (!connection_established()) {
        throw RemotePolicyError("No connection to remote policy established.\n"
                                "Make sure your FD call starts with --remote-policy <url>.");
    }
    // TODO (Jan) added this line, check if unpack is really necessary
    state_in.unpack();
    const std::vector<int> &state = state_in.get_values();
    int op_id =
        phrmPolicyFDRStateOperator(pheromone_policy, state.data(), state.size());
    if (op_id >= 0) {
        return OperatorID(op_id);
    } else if (op_id == OperatorID::no_operator_index) {
        return OperatorID::no_operator;
    } else {
        std::cerr << "phrmPolicyFDRStateOperator failed" << std::endl;
        utils::exit_with(utils::ExitCode::REMOTE_POLICY_ERROR);
    }
}

OperatorID PolicyClient::apply(const State &state_in) {
    return static_apply(state_in);
}

   

    
bool PolicyClient::static_operators_prob(const State &state_in, vector<OperatorID>* applicable_operator_ids, vector<float>* operator_probabilities){
    if (!connection_established()) {
    throw RemotePolicyError("No connection to remote policy established.\n"
                            "Make sure your FD call starts with --remote-policy <url>.");
    }
    // TODO (Jan) added this line, check if unpack is really necessary
    state_in.unpack();
    const std::vector<int> &state = state_in.get_values();

    int op_size = 0;
    int *op_ids = NULL;
    float *op_probs = NULL;
    int st = phrmPolicyFDRStateOperatorsProb(pheromone_policy, state.data(), state.size(),
                                            &op_size, &op_ids, &op_probs);

    assert(st == 0);

    for (int opi = 0; opi < op_size; ++opi){
        applicable_operator_ids->push_back(OperatorID(op_ids[opi]));
        operator_probabilities->push_back(op_probs[opi]);
    }

    if (op_ids != NULL)
        free(op_ids);
    if (op_probs != NULL)
        free(op_probs);

    return 0;    

}


bool PolicyClient::get_operators_prob(const State &state, vector<OperatorID>* applicable_operator_ids, vector<float>* operator_probabilities){
    return static_operators_prob(state, applicable_operator_ids, operator_probabilities);
}

}
