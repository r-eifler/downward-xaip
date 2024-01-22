#include "radius_tracker.h"


using namespace std;

RadiusTracker::RadiusTracker(float min_radius, float max_radius, float step_size):
    min_radius(min_radius), max_radius(max_radius), step_size(step_size) {

        int num_buckets = (max_radius - min_radius) / step_size;
        for(int r = 0; r <= num_buckets; r++){
            // cout << "--> " << r << endl;
            unordered_set<FrontierElem, HashFrontierElem> ns;
            frontier[r] = ns;
        }
    }

bool RadiusTracker::has_next_radius(){
    return current_radius < max_radius;
}

float RadiusTracker::next_radius(){
    if(current_radius == -1) {
        current_radius = min_radius;
    }
    else {
        current_radius += step_size;
    }
    return current_radius;
}

void RadiusTracker::add_frontier_state(float distance, StateID parent, OperatorID op) {
    // cout << "distance: " << distance << endl;

    if(distance > max_radius){
        return;
    }

    if(distance <= min_radius){
        frontier.at(0).insert(FrontierElem(parent, op));
        return;
    }

    double index = ((distance - min_radius) / step_size);

    int real_index = std::ceil(index);
    // cout << "real index: " << real_index << endl;
    frontier.at(real_index).insert(FrontierElem(parent, op));
}

unordered_set<FrontierElem, HashFrontierElem> RadiusTracker::get_frontier_states(float radius){
    double index = ((radius - min_radius) / step_size);
    index = std::round(index * 1000.0) / 1000.0;
    int real_index = std::ceil(index);
    return frontier[real_index];
}


uint RadiusTracker::get_frontier_size(float radius){
    double index = ((radius - min_radius) / step_size);
    index = std::round(index * 1000.0) / 1000.0;
    int real_index = std::ceil(index);
    return frontier[real_index].size();
}