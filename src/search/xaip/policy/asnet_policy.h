#pragma once

#include "../policy.h"
#include "asnets.h"

namespace options {
class Options;
class OptionParser;
} // namespace options

namespace policy_testing {
class ASNetPolicy : public Policy {
public:
    explicit ASNetPolicy(const options::Options &opts);
    static void add_options_to_parser(options::OptionParser &parser);

protected:
    void initialize() override;
    OperatorID apply(const State &state) override;

private:
    const std::string domain_pddl_;
    const std::string problem_pddl_;
    const std::string snapshot_;
    std::unique_ptr<ASNetInterface> asnet_;
    const bool less_output_;
};
} // namespace policy_testing
