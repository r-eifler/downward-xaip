#ifndef GOAl_SUBSET_H
#define GOAl_SUBSET_H


#include <boost/dynamic_bitset.hpp>
#include <vector>
#include <iostream>

using ulong = unsigned long;

namespace goalsubset {

class GoalSubset {

    private:

        boost::dynamic_bitset<> goals;

    public:

    GoalSubset();
    GoalSubset(size_t max_num_goals);
    GoalSubset(boost::dynamic_bitset<> goals);
    GoalSubset(size_t max_num_goals, size_t index);


    bool contains(size_t index) const{
        return goals[index];
    };

    void add(size_t index) {
        goals[index] = true;
    };

    bool is_subset_of(GoalSubset set) const{
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

    bool is_superset_of(GoalSubset set) const{
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

    bool is_strict_superset_of(GoalSubset set) const{
        if(goals.size() != set.size()){
            return false;
        }
        for(long unsigned int i = 0; i < goals.size(); i++){
            if(set.contains(i)  && !this->contains(i)){
                return false;
            }
        }
        return true && ! (set == *this);
    };

    size_t size() const {
        return goals.size();
    }

    size_t count() const {
        return goals.count();
    }

    void set(size_t index, bool value) { 
        goals[index] = value; 
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

    bool all() const{
        return goals.all();
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
