#ifndef GOAl_SUBSET_SPACE_H
#define GOAl_SUBSET_SPACE_H

#include <cassert>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <unordered_set>
#include "../goal_subsets/goal_subset.h"
#include "../goal_subsets/goal_subsets.h"
#include "../task_proxy.h"
#include "../abstract_task.h"

#include <deque>

class State;

namespace goalsubsetspace {

enum GoalSubsetStatus {UNDIFINED, SOLVABLE, UNSOLVABLE};

class GoalSpaceNode {
protected:
    std::vector<GoalSpaceNode*> children;
    uint sleep_set_i = 0;
    goalsubset::GoalSubset goals;
    GoalSubsetStatus status = UNDIFINED;
    bool printed = false;


public:

    GoalSpaceNode(goalsubset::GoalSubset goals);

    bool operator==(const GoalSpaceNode &other) const{ 
        return goals == other.goals;
    }

    void solved(bool propagate = false){
        status = SOLVABLE;
        if (propagate) {
            for(GoalSpaceNode* child : children){
                child->solved(true);
            }
        }
    }

    void not_solved(bool propagate = false){
        status = UNSOLVABLE;
        if (propagate) {
            for(GoalSpaceNode* child : children){
                child->not_solved(true);
            }
        }
    }

    bool statusDefined() const {
        return status != UNDIFINED;
    }

    bool isSolvable(){
        return status == SOLVABLE;
    }

    bool isUnSolvable(){
        return status == UNSOLVABLE;
    }

    void addChild(GoalSpaceNode* node){
        children.push_back(node);
    }

   
    void set_goals(goalsubset::GoalSubset goals){
        this->goals = goals;
    }

    goalsubset::GoalSubset get_goals() const{
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
    void print();
    std::vector<GoalSpaceNode*> weaken() const;
    std::vector<GoalSpaceNode*> strengthen() const;

};

class MyNodeHashFunction {
    public:

    std::size_t operator()(GoalSpaceNode* const n) const{
        return n->get_goals().get_id();
    }
};

class MyNodeEqualFunction {
    public:

    std::size_t operator()(GoalSpaceNode* const n1, GoalSpaceNode* const n2) const{
        return n1->get_goals() == n2->get_goals();
    }
};


class GoalSubsetSpace{

protected:
    bool weaken;

    // internal openlist of goal subset space
    std::deque<GoalSpaceNode*> open_list;

    // already generated subsets
    std::unordered_set<GoalSpaceNode*, MyNodeHashFunction, MyNodeEqualFunction> generated;

    // root node of the goal subset space
    GoalSpaceNode* root;
    GoalSpaceNode* current_node;

    std::vector<FactPair> soft_goal_list;
    std::vector<FactPair> hard_goal_list;

    std::vector<std::string> soft_goal_fact_names;

    /**
     * removes nodes from the openlist until one with an undefined status is found
     * @return the first node with undefined status
     */
    GoalSpaceNode* get_next_node();

    GoalSubsets generate_MUGS();
    GoalSubsets generate_MSGS();

public:

    GoalSubsetSpace(GoalsProxy goals, bool all_soft_goals, bool weaken);

    bool continue_search(){
        return ! open_list.empty();
    }

    GoalSpaceNode* get_root() const {
        return root;
    }

    GoalSpaceNode* get_current_node(){
        return current_node;
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
     * @return false if there is no next node true otherwise
     */
    bool next_node();

    /**
     * Converts the bit set representation to a variable value pair representation
     * @return list of contained goals as fact pairs
     */
    std::vector<FactPair> get_current_goals();

    void current_goals_solved(bool solved, bool propagate = false);
    void expand();

    void print();
     
};



}

#endif
