#ifndef GOAl_SUBSET_H
#define GOAl_SUBSET_H


#include <bitset>
#include <vector>
#include <iostream>

namespace goalsubset {

class GoalSubset {

    private:

        std::bitset<64> goals;
        int max_num_goals;

    public:

    GoalSubset();
    GoalSubset(int max_num_goals);
    GoalSubset(std::bitset<64> goals, int max_num_goals);
    GoalSubset(int goal_id, int max_num_goals);

    int max_elements(){
        return max_num_goals;
    }

    bool contains(int goal_id) const{
        return goals[goal_id];
    };

    bool isSubsetOf(GoalSubset set) const{
        if(max_num_goals != set.max_num_goals){
            return false;
        }
        for(int i = 0; i < max_num_goals; i++){
            if(!set.contains(i)  && this->contains(i)){
                return false;
            }
        }
        return true;
    };

    bool isSupersetOf(GoalSubset set) const{
        if(max_num_goals != set.max_num_goals){
            return false;
        }
        for(int i = 0; i < max_num_goals; i++){
            if(set.contains(i)  && !this->contains(i)){
                return false;
            }
        }
        return true;
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

    std::vector<GoalSubset> singelten_subsets() const;
    GoalSubset complement() const;
    GoalSubset set_union(GoalSubset set) const;
    GoalSubset set_intersection(GoalSubset set) const;
    

    void print() const;
};

class GoalSubsetHashFunction {
    public:

    std::size_t operator()(GoalSubset const n) const{
        return n.get_id();
    }
};

class GoalSubsetEqualFunction {
    public:

    std::size_t operator()(GoalSubset const s1, GoalSubset const s2) const{
        return s1.get_id() == s2.get_id();
    }
};
}


#endif