#ifndef POLICY_CLIENT_H
#define POLICY_CLIENT_H

#include "../../operator_id.h"
#include "../../search_space.h"
#include "../utils/exceptions.h"

#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

#include <pheromone/policy_client.h>

namespace policy {

class RemotePolicyError : public utils::Exception {
    std::string msg;
public:
    explicit RemotePolicyError(std::string msg);

    void print() const override;
};


class PolicyClient{

private:
    std::string url = "localhost:8888";
    int sockfd;
    struct sockaddr_in servaddr; 

    bool project_resources = true;

    std::shared_ptr<AbstractTask> task;

    inline static phrm_policy_t *pheromone_policy = nullptr;
    inline static std::shared_ptr<PolicyClient> g_default_policy = nullptr;

    // void send_state(const State & state);
    // std::vector<double> process_reponse(std::vector<OperatorID> & operator_ids);
    

public:
    explicit PolicyClient(const std::string url, bool project_resoruces);

    void initialize(const std::shared_ptr<AbstractTask> &task);

    // std::vector<double> get_value(const State & state, std::vector<OperatorID> & operators);

    /**
     * Establishes a connection to the remote server.
     */
    static void establish_connection(const std::string &url);

    /**
     * Establishes a connection to the remote server.
     */
    static bool connection_established() {return pheromone_policy;}

    static std::shared_ptr<PolicyClient> get_global_default_policy();

    /**
     * Returns FDR planning task in the Fast Downward format
     * https://www.fast-downward.org/TranslatorOutputFormat
     */
    static std::string input_fdr();

    /**
     * Apply policy on the state and retrieve the selected operator.
     */
    OperatorID apply(const State &state);
    static OperatorID static_apply(const State &state);
};
}

#endif
