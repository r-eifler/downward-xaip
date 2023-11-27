#ifndef PRUNING_POLICY_CONFIDENCE_PRUNING_METHOD_H
#define PRUNING_POLICY_CONFIDENCE_PRUNING_METHOD_H

#include "../../pruning_method.h"
#include "policy_client.h"

namespace policy_pruning_method {
class PolicyConfidencePruningMethod : public PruningMethod {

    std::string url;
    bool project_resources = true;
    policy::PolicyClient policy_client;

    double T = 0.0;

    double sum_max_probabilities = 0;
    int num_tested_states = 0;
    std::vector<int> max_prob_distribution{0,0,0,0,0,0,0,0,0,0,0};

    virtual void prune(
        const State &, std::vector<OperatorID> &) override;
public:
    explicit PolicyConfidencePruningMethod(const options::Options &opts);
    virtual void initialize(const std::shared_ptr<AbstractTask> &) override;
    virtual void print_statistics() const override {
        std::cout << "Average maximal confidence: " << sum_max_probabilities / num_tested_states << std::endl;
        std::cout << "Num tested states: " << num_tested_states << std::endl;
        std::cout << "Maximal probability distribution: ";
        for(size_t i = 0; i < max_prob_distribution.size(); i++){
            std::cout << double(max_prob_distribution[i]) / num_tested_states << " ";
        }
        std::cout << std::endl;
    }
};
}

#endif
