#ifndef GOAl_SUBSET_SPACE_H
#define GOAl_SUBSET_SPACE_H

#include <cassert>
#include <memory>
#include <utility>
#include <vector>
#include <unordered_set>
#include "../goal_subsets/goal_subset.h"
#include "../task_proxy.h"
#include "../abstract_task.h"

#include <deque>

class State;

namespace goalsubsetspace {
class GoalSpaceNode;


class GoalSubsetSpace{

protected:
    bool weaken;

    // internal openlist of goal subset space
    std::deque<GoalSpaceNode*> open_list;

    // already visited subsets
    std::unordered_set<GoalSpaceNode*> visisted;

    // root node of the goal subset space
    GoalSpaceNode* root;
    GoalSpaceNode* current_node;

    std::vector<FactPair> soft_goal_list;
    std::vector<FactPair> hard_goal_list;

    // all nodes of the goal subset space
    std::vector<GoalSpaceNode*> nodes;

public:

    GoalSubsetSpace(GoalsProxy goals, bool all_soft_goals, bool weaken);

    bool continue_search(){
        return ! open_list.empty();
    }

    /**
     * removes the first node from the openlist and returns it
     * @return the first node in the openlist
     */
    GoalSpaceNode* get_next_node(){
        GoalSpaceNode* next_node = open_list.front();
        open_list.pop_front();
        return next_node;
    }

    GoalSpaceNode* get_root() const {
        return root;
    }

    std::vector<FactPair> getSoftGoals() {
        return soft_goal_list;
    }

    std::vector<FactPair> getHardGoals() {
        return hard_goal_list;
    }

    /**
     * Get the goal facts which are in the node @node
     * @param node
     * @return
     */
    std::vector<FactPair> get_goals(const GoalSpaceNode* node) const;

    /**
     * updates the current_node with the node returned by get_next_node
     */
    void next_node();

    /**
     * TODO
     * @return
     */
    std::vector<FactPair> get_next_goals();
    void current_goals_solved(bool solved);
    void expand();
    int print_relation();
    void print();
     
};


class GoalSpaceNode {
protected:
    std::vector<GoalSpaceNode*> children;
    uint sleep_set_i = 0;
    goalsubset::GoalSubset goals;
    // uint goals = (1U << 31);
    bool solvable = false;
    bool printed = false;


public:

    GoalSpaceNode(goalsubset::GoalSubset goals);

    bool operator==(const GoalSpaceNode &other) const{ 
        return goals == other.goals;
    }

    std::size_t operator()(const GoalSpaceNode& n) const{
        return n.goals.get_id();
    }

    void solved(){
        solvable = true;
    }

    void not_solved(){
        solvable = false;
    }

    bool isSolvable(){
        return solvable;
    }

    void addChild(GoalSpaceNode* node){
        children.push_back(node);
    }

   
    void set_goals(goalsubset::GoalSubset goals){
        this->goals = goals;
    }

    goalsubset::GoalSubset get_goals(){
        return goals;
    }

    void set_sleep_set_id(uint id){
        this->sleep_set_i = id;
    }

    bool hasChildren(){
        return children.size() > 0;
    }

    std::vector<GoalSpaceNode*> getChildren(){
        return children;
    }

    void resetPrinted(){
        printed = false;
    }

    std::vector<FactPair>  get_goals(const std::vector<FactPair>& all_goals) const;
    void print(const std::vector<FactPair>& all_goals);
    std::vector<GoalSpaceNode*> weaken() const;
    std::vector<GoalSpaceNode*> strengthen() const;

};
}

#endif
