#include "distance_function.h"

#include "../../option_parser.h"
#include "../../plugin.h"

#include "../../utils/logging.h"

#include <cassert>

using namespace std;

DistanceFunction::DistanceFunction(const options::Options &opts) :
      log(utils::get_log_from_options(opts)),
      task(nullptr) {
}

void DistanceFunction::initialize(const shared_ptr<AbstractTask> &task_) {
    assert(!task);
    task = task_;
}

vector<float> DistanceFunction::get_distances(const State &, std::vector<OperatorID> &op_ids) {

    vector<float> distances;
    for(uint i = 0; i < op_ids.size(); i++){
            distances.push_back(0);
    }

    return distances;
}

void DistanceFunction::print_statistics() const {
   
}

static PluginTypePlugin<DistanceFunction> _type_plugin(
    "DistanceFunction",
    "TODO");
