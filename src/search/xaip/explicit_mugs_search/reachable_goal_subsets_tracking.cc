#include "reachable_goal_subsets_tracking.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;
using namespace goalsubset;

namespace reachable_goal_subsets_tracking {
ReachableGoalSubsetsTracking::ReachableGoalSubsetsTracking(const Options &opts)
    : PruningMethod(opts){

    log << "--> reachable goal subset tracking" << endl;
    
}

void ReachableGoalSubsetsTracking::initialize(const shared_ptr<AbstractTask> &task) {
    if (initialized)
        return;

    PruningMethod::initialize(task);

    current_msgs = MSGSCollection();
    current_msgs.initialize(task);

    log << "initialize pruning method: reachable goal subset tracking" << endl;
    initialized = true;
}


bool ReachableGoalSubsetsTracking::prune(const State &state, int){

    current_msgs.track(state);

    return false;
}

void ReachableGoalSubsetsTracking::print_statistics() const {
    current_msgs.print();
}

MSGSCollection ReachableGoalSubsetsTracking::get_msgs() const {
    return current_msgs;
}

void ReachableGoalSubsetsTracking::init_msgs(MSGSCollection subsets) {
    current_msgs = subsets;
}


static shared_ptr<PruningMethod> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "reachable goal subsets",
        "States are pruned if no subset of goals already seen is reachable");
    add_pruning_options_to_parser(parser);

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<ReachableGoalSubsetsTracking>(opts);
}

static Plugin<PruningMethod> _plugin("rgsst", _parse);
}
