#include "asnet_policy.h"

#include "../../option_parser.h"
#include "../../plugin.h"

#include <iostream>
#include <memory>
#include <vector>

namespace policy_testing {
ASNetPolicy::ASNetPolicy(const options::Options &opts)
    : Policy(opts)
      , domain_pddl_(opts.get<std::string>("domain_pddl"))
      , problem_pddl_(opts.get<std::string>("problem_pddl"))
      , snapshot_(opts.get<std::string>("snapshot"))
      , asnet_(nullptr)
      , less_output_(opts.get<bool>("less_output")) {
}

void
ASNetPolicy::initialize() {
    if (initialized) {
        assert(asnet_);
        return;
    }
    if (less_output_) {
        std::cout.setstate(std::ios_base::failbit);
        std::cerr.setstate(std::ios_base::failbit);
    }
    asnet_ = std::make_unique<ASNetInterface>(domain_pddl_, problem_pddl_, snapshot_);
    if (less_output_) {
        std::cout.clear();
        std::cerr.clear();
    }
    Policy::initialize();
}

void
ASNetPolicy::add_options_to_parser(options::OptionParser &parser) {
    Policy::add_options_to_parser(parser);
    parser.add_option<std::string>("domain_pddl", "Domain PDDL", "");
    parser.add_option<std::string>("problem_pddl", "Problem PDDL", "");
    parser.add_option<std::string>("snapshot", "Snapshot .pkl file", "");
    parser.add_option<bool>("less_output", "", "true");
}

OperatorID
ASNetPolicy::apply(const State &state) {
    std::vector<OperatorID> applicable_ops;
    generate_applicable_ops(state, applicable_ops);
    if (applicable_ops.empty()) {
        return NO_OPERATOR;
    }
    if (less_output_) {
        std::cout.setstate(std::ios_base::failbit);
        std::cerr.setstate(std::ios_base::failbit);
    }
    const OperatorID result = asnet_->apply_policy(state, applicable_ops);
    if (less_output_) {
        std::cout.clear();
        std::cerr.clear();
    }
    return result;
}

static Plugin<Policy> _plugin("asnet_policy", options::parse<Policy, ASNetPolicy>);
} // namespace policy_testing
