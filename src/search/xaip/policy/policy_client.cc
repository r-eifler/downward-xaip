#include "policy_client.h"

#include "../option_parser.h"
#include "../plugin.h"

#include <boost/algorithm/string/join.hpp>
#include <string>

using namespace std;

namespace policy {

    PolicyClient::PolicyClient(const int port): port(port) {
    
        // Creating socket file descriptor 
        if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
            perror("socket creation failed"); 
            exit(EXIT_FAILURE); 
        } 
    
        memset(&servaddr, 0, sizeof(servaddr)); 
        
        // Filling server information 
        servaddr.sin_family = AF_INET; 
        servaddr.sin_port = htons(port); 
        servaddr.sin_addr.s_addr = INADDR_ANY; 

    }

    void PolicyClient::initialize(const shared_ptr<AbstractTask> &task_) {
        assert(!task);
        task = task_;
    }

bool not_needed_characters(char c){
    return c == ' ' || c == ',' || c == ')' || c == '=';
} 


vector<double> PolicyClient::get_value(const State & state, vector<OperatorID> & operator_ids){

    TaskProxy task_proxy = TaskProxy(*task);

    // for(int i = 0; i < operator_ids.size(); i++){
    //     cout << operator_ids[i] << endl;
    //     cout << task_proxy.get_operators()[operator_ids[i]].get_name() << endl;
    // }

    vector<string> atoms;

    for(size_t i = 0; i < state.size(); i++){
        if(state[i].get_name().rfind("Atom", 0) == 0){
            string atom = state[i].get_name().replace(0,5,"");
            atoms.push_back(atom);
        }
    }
    
    const string state_string = boost::algorithm::join(atoms, "|");
    // cout << "State: " << state_string << endl;
    const char *state_data = state_string.c_str(); 

    char buffer[1024]; 
    int n;
    socklen_t len; 
    
    sendto(sockfd, (const char *)state_data, strlen(state_data), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr)); 
        
    n = recvfrom(sockfd, (char *)buffer, 1024, MSG_WAITALL, (struct sockaddr *) &servaddr, &len); 
    buffer[n] = '\0'; 
    // std::cout<<"Policy: "<<buffer<<std::endl; 
    std::string message(buffer);

    std::vector<double> policy_values;

    //parse message
    int pos_action = 0;
    int end = 0;
    while(end != -1){
        end = message.find("|", pos_action);
        if(end == -1){
            break;
        }
        std::string action = message.substr(pos_action, end - pos_action);
        // std::cout << "Action: " << action << std::endl;
        int pos_space = action.find("=",0);
        std::string name = action.substr(0, pos_space-1);
        // std::cout << "Name: " << name << std::endl;
        std::string value = action.substr(pos_space + 1, end - pos_space);
        // std::cout << "Value: " << value << std::endl;

        for(size_t i = 0; i < operator_ids.size(); i++){
            if(task_proxy.get_operators()[operator_ids[i]].get_name() == name){
                cout << task_proxy.get_operators()[operator_ids[i]].get_name() << " " << value << endl;
                policy_values.push_back(std::stod(value));
            }
        }

        pos_action = end + 1;
}

    // close(sockfd); 
    return policy_values;
}

}
