#ifndef FAST_DOWNWARD_TASK_RELAXATION_TRACKER_H
#define FAST_DOWNWARD_TASK_RELAXATION_TRACKER_H

#include "../../open_list.h"

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

    unordered_map<float,unordered_set<StateID>> frontier;

public:
    RadiusTracker(float min_radius, float max_radius, float step_size);

    bool has_next_radius();
    float next_radius();

    void add_frontier_state(float distance, StateID id) {
        if(distance > max_radius){
            return;
        }

        float min_included_radius = min_radius;
        while(min_included_radius <= max_radius && distance > min_included_radius){
            min_included_radius += step_size;
        }

        frontier.at(min_included_radius).insert(id);
    }

    unordered_set<StateID> get_frontier_states(float radius){
        return frontier[radius];
    }
    
};


#endif //FAST_DOWNWARD_TASK_RELAXATION_TRACKER_H
