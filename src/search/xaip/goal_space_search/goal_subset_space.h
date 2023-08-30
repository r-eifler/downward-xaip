#ifndef GOAl_SUBSET_SPACE_H
#define GOAl_SUBSET_SPACE_H

#include <cassert>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
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
    std::vector<GoalSpaceNode*> subsets;
    std::vector<GoalSpaceNode*> supersets;
    goalsubset::GoalSubset goals;
    GoalSubsetStatus status = UNDIFINED;
    bool printed = false;


public:

    GoalSpaceNode(goalsubset::GoalSubset goals);

    bool operator==(const GoalSpaceNode &other) const{ 
        return goals == other.goals;
    }

    void solved(bool propagate = false){
        assert(!statusDefined());
        status = SOLVABLE;
        if (propagate) {
            for(GoalSpaceNode* subset : subsets){
                subset->solved(true);
            }
        }
    }

    void not_solved(bool propagate = false){
        assert(!statusDefined());
        status = UNSOLVABLE;
        if (propagate) {
            for(GoalSpaceNode* superset : supersets){
                superset->not_solved(true);
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

    void add_subset(GoalSpaceNode* node){
        subsets.push_back(node);
    }

    void add_superset(GoalSpaceNode* node){
        supersets.push_back(node);
    }

   
    void set_goals(goalsubset::GoalSubset goals){
        this->goals = goals;
    }

    goalsubset::GoalSubset get_goals() const{
        return goals;
    }

    bool has_subsets(){
        return subsets.size() > 0;
    }

    bool has_supersets(){
        return supersets.size() > 0;
    }

    std::vector<GoalSpaceNode*> get_supersets(){
        return supersets;
    }

     std::vector<GoalSpaceNode*> get_subsets(){
        return subsets;
    }

    void resetPrinted(){
        printed = false;
    }

    size_t size(){
        return goals.size();
    }

    size_t count(){
        return goals.count();
    }

    std::vector<FactPair>  get_goals(const std::vector<FactPair>& all_goals) const;
    void print(const std::vector<FactPair>& all_goals);
    void print();
    std::vector<GoalSpaceNode*> weaken(std::unordered_map<int, goalsubset::GoalSubset>* next_weaker) const;
    std::vector<GoalSpaceNode*> strengthen(std::unordered_map<int, goalsubset::GoalSubset>* next_stronger, goalsubset::GoalSubset weakest,  std::unordered_map<int, goalsubset::GoalSubset*>* connected_soft_goals) const;

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

    GoalSpaceNode* current_node;

    std::vector<FactPair> soft_goal_list;
    std::vector<FactPair> hard_goal_list;

    std::vector<std::string> soft_goal_fact_names;

    std::unordered_map<int, goalsubset::GoalSubset> next_weaker;
    std::unordered_map<int, goalsubset::GoalSubset> next_stronger;
    std::unordered_map<int, goalsubset::GoalSubset*> connected_soft_goals;
    goalsubset::GoalSubset strongest;
    goalsubset::GoalSubset weakest;

    void init_soft_goal_relations();

    void init_root_node();

    GoalSubsets generate_MUGS();
    GoalSubsets generate_MSGS();

public:

    GoalSubsetSpace(GoalsProxy goals, bool all_soft_goals, bool weaken);

    bool continue_search(){
        return ! open_list.empty();
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
     * looks for the next node whose status can not be derived
     * @return true if there is a node with undefined states false otherwise
     */
    bool next_node_to_test();

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
