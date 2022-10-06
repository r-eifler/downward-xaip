#ifndef GOAl_SUBSET_H
#define GOAl_SUBSET_H


#include <bitset>
#include <vector>
#include <iostream>

namespace goalsubset {

class GoalSubset {

    private:

        std::bitset<64> goals;

    public:

    GoalSubset();
    GoalSubset(std::bitset<64> goals);

    bool contains(int goal_id) const{
        return goals[goal_id];
    };

    size_t size() const {
        return goals.size();
    }

    bool operator==(const GoalSubset &other) const{ 
        return goals == other.goals;
    }

    std::size_t operator()(const GoalSubset& s) const{
        return (size_t) s.goals.to_ulong();
    };

    ulong get_id() const{
        return goals.to_ulong();
    }

    bool is_empty() const{
        return goals.to_ulong() == 0;
    }

    std::vector<GoalSubset> weaken() const;
    std::vector<GoalSubset> strengthen() const;
    void print() const;
};
}


#endif