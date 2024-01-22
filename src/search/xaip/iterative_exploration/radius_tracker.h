#ifndef FAST_DOWNWARD_TASK_RELAXATION_TRACKER_H
#define FAST_DOWNWARD_TASK_RELAXATION_TRACKER_H

#include "../../open_list.h"
#include "frontier_elem.h"

#include <string>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

using namespace std;

class RadiusTracker {

private:
    
    float min_radius;
    float max_radius;
    float step_size;

    float current_radius = -1;

    unordered_map<float,unordered_set<FrontierElem, HashFrontierElem>> frontier;

public:
    RadiusTracker(float min_radius, float max_radius, float step_size);

    bool has_next_radius();
    float next_radius();

    void add_frontier_state(float distance, StateID parent, OperatorID op);

    unordered_set<FrontierElem, HashFrontierElem> get_frontier_states(float radius);
    uint get_frontier_size(float radius);
    
};


#endif //FAST_DOWNWARD_TASK_RELAXATION_TRACKER_H
