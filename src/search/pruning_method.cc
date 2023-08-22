#include "pruning_method.h"

#include "option_parser.h"
#include "plugin.h"

#include "task_utils/task_properties.h"

#include "utils/logging.h"

#include <cassert>

using namespace std;

PruningMethod::PruningMethod(const options::Options &opts)
    : timer_ops(false),
      timer_states(false),
      log(utils::get_log_from_options(opts)),
      task(nullptr) {
}

void PruningMethod::initialize(const shared_ptr<AbstractTask> &task_) {
    assert(!task);
    task = task_;
    num_successors_before_pruning = 0;
    num_successors_after_pruning = 0;
    num_pruned_states = 0;
}

void PruningMethod::prune_operators(
    const State &state, vector<OperatorID> &op_ids) {
    assert(!task_properties::is_goal_state(TaskProxy(*task), state));
    /*
      We only measure time with verbose verbosity level because time
      measurements induce a significant overhead, up to 30% for configurations
      like blind search. See issue1042 and issue1058.
    */
    if (log.is_at_least_verbose()) {
        timer_ops.resume();
    }
    int num_ops_before_pruning = op_ids.size();
    prune(state, op_ids);
    num_successors_before_pruning += num_ops_before_pruning;
    num_successors_after_pruning += op_ids.size();
    if (log.is_at_least_verbose()) {
        timer_ops.stop();
    }
}

bool PruningMethod::prune_state(const State &state, int remaining_cost){
    assert(!task_properties::is_goal_state(TaskProxy(*task), state));
    /*
      We only measure time with verbose verbosity level because time
      measurements induce a significant overhead, up to 30% for configurations
      like blind search. See issue1042 and issue1058.
    */
    if (log.is_at_least_verbose()) {
        timer_states.resume();
    }
    bool can_be_pruned = prune(state, remaining_cost);
    num_tested_states++;
    num_pruned_states = can_be_pruned ? num_pruned_states + 1 : num_pruned_states;
    if (log.is_at_least_verbose()) {
        timer_states.stop();
    }
    return can_be_pruned;
}

bool PruningMethod::prune(const State &, int){
    return false;
}

StateID PruningMethod::get_cardinally_best_state(){
    cout << "OSP cardinally best state not implemented" << endl;
    return StateID::no_state;
}

void PruningMethod::print_statistics() const {
    if (log.is_at_least_normal()) {
        log << "total successors before pruning: "
            << num_successors_before_pruning << endl
            << "total successors after pruning: "
            << num_successors_after_pruning << endl;
        double pruning_ratio_ops = (num_successors_before_pruning == 0) ? 1. : 1. - (
            static_cast<double>(num_successors_after_pruning) /
            static_cast<double>(num_successors_before_pruning));
        log << "Pruning ratio operators: " << pruning_ratio_ops << endl;
        if (log.is_at_least_verbose()) {
            log << "Time for pruning operators: " << timer_ops << endl;
        }

        log << "total tested states: "
            << num_tested_states << endl
            << "total pruned states: "
            << num_pruned_states << endl;
        double pruning_ratio_states = (num_tested_states == 0) ? 1. : 1. - (
            static_cast<double>(num_pruned_states) /
            static_cast<double>(num_tested_states));
        log << "Pruning ratio states: " << pruning_ratio_states << endl;
        if (log.is_at_least_verbose()) {
            log << "Time for pruning states: " << timer_states << endl;
        }
    }
}

void PruningMethod::set_abstract_task(std::shared_ptr<AbstractTask> t){
    task = t;
}

MSGSCollection PruningMethod::get_msgs() const{
    MSGSCollection empty;
    return empty;
}

void PruningMethod::init_msgs(MSGSCollection collection){
    std::cout << "Number of msgs" << collection.size() << std::endl;
}


void add_pruning_options_to_parser(options::OptionParser &parser) {
    utils::add_log_options_to_parser(parser);
    parser.document_note(
        "Note on verbosity parameter",
        "Setting verbosity to verbose or higher enables time measurements "
        "in each call to prune_operators for a given state. This induces a "
        "significant overhead, up to 30% in configurations like blind search "
        "with the no pruning method (`null`). We recommend using at most "
        "normal verbosity for running experiments.");
}

static PluginTypePlugin<PruningMethod> _type_plugin(
    "PruningMethod",
    "Prune or reorder applicable operators.");
