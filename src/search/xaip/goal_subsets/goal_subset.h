#ifndef GOAl_SUBSET_H
#define GOAl_SUBSET_H


#include <boost/dynamic_bitset.hpp>
#include <vector>
#include <iostream>

namespace goalsubset {

class GoalSubset {

    private:

        boost::dynamic_bitset<> goals;

    public:

    GoalSubset();
    GoalSubset(int max_num_goals);
    GoalSubset(boost::dynamic_bitset<> goals);
    GoalSubset(int goal_id, int max_num_goals);


    bool contains(int goal_id) const{
        return goals[goal_id];
    };

    bool isSubsetOf(GoalSubset set) const{
        if(goals.size() != set.size()){
            return false;
        }
        for(long unsigned int i = 0; i < goals.size(); i++){
            if(!set.contains(i)  && this->contains(i)){
                return false;
            }
        }
        return true;
    };

    bool isSupersetOf(GoalSubset set) const{
        if(goals.size() != set.size()){
            return false;
        }
        for(long unsigned int i = 0; i < goals.size(); i++){
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
        return goals.none();
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