#include "policy_client.h"

#include "../option_parser.h"
#include "../plugin.h"

#include <boost/algorithm/string/join.hpp>
#include <string>

using namespace std;

namespace policy {

    PolicyClient::PolicyClient(const int port, bool project_resources): 
    port(port), project_resources(project_resources){
    
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

void PolicyClient::send_state(const State & state) {
    vector<string> atoms;

    for(size_t i = 0; i < state.size(); i++){
        if(state[i].get_name().rfind("Atom", 0) == 0){
            string atom = state[i].get_name().replace(0,5,"");
            if(! project_resources || atom.find("budget",0) == std::string::npos){
                atoms.push_back(atom);
            }  
        }
    }
    
    const string state_string = boost::algorithm::join(atoms, "|");
    // cout << "State: " << state_string << endl;
    const char *state_data = state_string.c_str(); 
    
    sendto(sockfd, (const char *)state_data, strlen(state_data), MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr)); 
}

std::vector<double> PolicyClient::process_reponse(vector<OperatorID> & operator_ids){

    char buffer[1024]; 
    int n;
    socklen_t len;  
    n = recvfrom(sockfd, (char *)buffer, 1024, MSG_WAITALL, (struct sockaddr *) &servaddr, &len); 
    buffer[n] = '\0'; 
    // std::cout<<"Policy: "<<buffer<<std::endl; 
    std::string message(buffer);

    std::vector<double> policy_values;

    //parse message
    TaskProxy task_proxy = TaskProxy(*task);
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

            string operator_name = task_proxy.get_operators()[operator_ids[i]].get_name();
            // cout << operator_name << endl;

            if(project_resources){
                int space1 = operator_name.find(" ", 0);
                int space2 = operator_name.find(" ", space1+1);
                int space3 = operator_name.find(" ", space2+1);
                operator_name = operator_name.substr(0, space1) + operator_name.substr(space3, message.size() - space3);
                // cout << "removed budget: " <<  operator_name << endl;
            }

            if(operator_name == name){
                //  cout << "Value: " << operator_name << " " << value << endl;
                policy_values.push_back(std::stod(value));
            }
        }

        pos_action = end + 1;
    }

    // close(sockfd); 
    return policy_values;
}


vector<double> PolicyClient::get_value(const State & state, vector<OperatorID> & operator_ids){

    send_state(state);

    return process_reponse(operator_ids);
}

}
