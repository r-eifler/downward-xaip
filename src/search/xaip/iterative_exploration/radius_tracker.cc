#include "radius_tracker.h"


using namespace std;

RadiusTracker::RadiusTracker(float min_radius, float max_radius, float step_size):
    min_radius(min_radius), max_radius(max_radius), step_size(step_size) {

        for(float r = min_radius; r <= max_radius; r += step_size){
            unordered_set<StateID> ns;
            frontier[r] = ns;
        }
    }

bool RadiusTracker::has_next_radius(){
    return current_radius < max_radius;
}

float RadiusTracker::next_radius(){
    if(current_radius == 1) {
        current_radius = min_radius;
    }
    else {
        current_radius += step_size;
    }
    return current_radius;
}