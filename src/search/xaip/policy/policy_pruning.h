#ifndef PRUNING_NULL_PRUNING_METHOD_H
#define PRUNING_NULL_PRUNING_METHOD_H

#include "../../pruning_method.h"
#include "policy_client.h"

namespace policy_pruning_method {
class PolicyPruningMethod : public PruningMethod {

    int port;
    policy::PolicyClient policy_client;

    double T = 0.0;

    virtual void prune(
        const State &, std::vector<OperatorID> &) override;
public:
    explicit PolicyPruningMethod(const options::Options &opts);
    virtual void initialize(const std::shared_ptr<AbstractTask> &) override;
    virtual void print_statistics() const override {}
};
}

#endif
