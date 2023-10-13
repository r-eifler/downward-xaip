#ifndef PRUNING_POLICY_CONFIDENCE_PRUNING_METHOD_H
#define PRUNING_POLICY_CONFIDENCE_PRUNING_METHOD_H

#include "../../pruning_method.h"
#include "policy_client.h"

namespace policy_pruning_method {
class PolicyConfidencePruningMethod : public PruningMethod {

    int port;
    bool project_resources = true;
    policy::PolicyClient policy_client;

    double T = 0.0;

    virtual void prune(
        const State &, std::vector<OperatorID> &) override;
public:
    explicit PolicyConfidencePruningMethod(const options::Options &opts);
    virtual void initialize(const std::shared_ptr<AbstractTask> &) override;
    virtual void print_statistics() const override {}
};
}

#endif
