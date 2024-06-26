#include "goal_subset_space.h"

#include "../task_proxy.h"
#include "../tasks/root_task.h"

#include "../utils/system.h"

#include <boost/dynamic_bitset.hpp>

using namespace std;
using namespace options;
using namespace goalsubset;

namespace goalsubsetspace {

GoalSpaceNode::GoalSpaceNode(GoalSubset goals):
    goals(goals){}

vector<FactPair> GoalSpaceNode::get_goals(const vector<FactPair>& all_goals) const{
    assert(all_goals.size() == goals.size());

    vector<FactPair> res;

    for (uint i = 0; i < all_goals.size(); i++) {
        if (goals.contains(i)) {
            res.push_back(all_goals[i]);
        }
    }
    return res;
}

void printList(vector<FactPair> list){
    for(FactPair l : list){
        cout << "var" << l.var << " = " << l.value << ", ";
    }
    cout << endl;
}

vector<GoalSpaceNode*> GoalSpaceNode::weaken() const {
    vector<GoalSubset> new_subsets = goals.weaken();
    vector<GoalSpaceNode*> new_nodes;
    for(GoalSubset subset : new_subsets){
        new_nodes.push_back(new GoalSpaceNode(subset));
    }
    return new_nodes;
}

vector<GoalSpaceNode*> GoalSpaceNode::strengthen() const{
    vector<GoalSubset> new_subsets = goals.strengthen();
    vector<GoalSpaceNode*> new_nodes;
    for(GoalSubset subset : new_subsets){
        new_nodes.push_back(new GoalSpaceNode(subset));
    }
    return new_nodes;
}

void GoalSpaceNode::print(const vector<FactPair>& all_goals){

    TaskProxy taskproxy = TaskProxy(*tasks::g_root_task.get());

    for(uint i = 0; i < get_goals(all_goals).size(); i++){
        FactPair g = get_goals(all_goals)[i];
        cout << taskproxy.get_variables()[g.var].get_fact(g.value).get_name();
        if(i < get_goals(all_goals).size()-1){
            cout << "|";
        }
    }

}

void GoalSpaceNode::print(){

    cout << "-------------------" << endl;
    goals.print();
    cout << "status: " << status << endl;
    cout << "-------------------" << endl;
}

GoalSubsetSpace::GoalSubsetSpace(GoalsProxy goals, bool all_soft_goals, bool weaken):
    weaken(weaken){

    TaskProxy task_proxy = TaskProxy(*tasks::g_root_task.get());

    if(all_soft_goals){
        for(uint i = 0; i < goals.size(); i++){
            soft_goal_list.push_back(goals[i].get_pair());
        }
    }
    else{
        for(uint i = 0; i < task_proxy.get_soft_goals().size(); i++){
            soft_goal_list.push_back(task_proxy.get_soft_goals()[i].get_pair());
        }
        for(uint i = 0; i < task_proxy.get_hard_goals().size(); i++){
            hard_goal_list.push_back(task_proxy.get_hard_goals()[i].get_pair());
        }
    }

    cout << "#soft goals:" << soft_goal_list.size() << endl;

    //sort goal facts according to there variable id
    std::sort(soft_goal_list.begin(), soft_goal_list.end());

    soft_goal_fact_names = std::vector<std::string>(soft_goal_list.size());
    for(uint i = 0; i < soft_goal_list.size(); i++){
        FactPair gp = soft_goal_list[i];
        soft_goal_fact_names[i] = task_proxy.get_variables()[gp.var].get_fact(gp.value).get_name();
    }

    boost::dynamic_bitset<> init_goals = weaken ? 
        boost::dynamic_bitset<>(soft_goal_list.size(), (1U << soft_goal_list.size()) - 1) : 
        boost::dynamic_bitset<>(soft_goal_list.size(), 0);
    root = new GoalSpaceNode(GoalSubset(init_goals));
    current_node = root;
    open_list.push_back(root); 
    generated.insert(root);

    cout << "Initial goal subset: " << endl;
    current_node->print();
}

 void GoalSubsetSpace::current_goals_solved(bool solved, bool propagate){
        if (solved)
            current_node->solved(propagate);
        else
            current_node->not_solved(propagate);
    }


void GoalSubsetSpace::expand(){
    // cout << "------------ EXPAND ------------" << endl;

    vector<GoalSpaceNode*> new_nodes = weaken ? current_node->weaken() : current_node->strengthen();
    
    // cout << "generated:"  << endl;
    // for (GoalSpaceNode* node : generated){
    //     node->get_goals().print();
    // }
    // cout << "children:"  << endl;
    //TODO implement a more efficient version
    for (GoalSpaceNode* node : new_nodes){
        // node->get_goals().print();
        auto existing_node_it = generated.find(node);
        if(existing_node_it != generated.end()){

            // new node not needed use existing copy
            delete node;

            // cout << "--> already exists"  << endl;
            // (*existing_node_it)->print();

            if (weaken){
                current_node->add_subset(*existing_node_it);
                (*existing_node_it)->add_superset(current_node);
            }
            else{
                current_node->add_superset(*existing_node_it);
                (*existing_node_it)->add_subset(current_node);
            }

            // propagate status
            if(weaken && current_node->isSolvable()){
                // cout << "is solvable" << endl;
                (*existing_node_it)->solved();
            }
            else if(!weaken && current_node->isUnSolvable()){
                // cout << "is not solvable" << endl;
                (*existing_node_it)->not_solved();
            }
            
        }
        else{
            // cout << "--> new node"  << endl;
            if (weaken){
                current_node->add_subset(node);
                (node)->add_superset(current_node);
            }
            else{
                current_node->add_superset(node);
                (node)->add_subset(current_node);
            }
            // cout << "--> links updated"  << endl;
            generated.insert(node);

            if(weaken && current_node->isSolvable()){
                // cout << "is solvable" << endl;
                node->solved();
            }
            else if(!weaken && current_node->isUnSolvable()){
                // cout << "is not solvable" << endl;
                node->not_solved();
            }

            open_list.push_back(node);

        }
    }
    // cout << "------------ EXPAND ------------" << endl;
}

vector<FactPair> GoalSubsetSpace::get_goals(const GoalSpaceNode* node) const
{
    vector<FactPair> current_goals;
    vector<FactPair> soft_goals = node->get_goals(soft_goal_list);
    current_goals.insert( current_goals.end(), hard_goal_list.begin(), hard_goal_list.end() );
    current_goals.insert( current_goals.end(), soft_goals.begin(), soft_goals.end() );
    return current_goals;
}

bool GoalSubsetSpace::next_node_to_test(){
    while(!open_list.empty()){
        GoalSpaceNode* next_node = open_list.front();
        open_list.pop_front();
        current_node = next_node;
        if(next_node->statusDefined()){
           this->expand();
        }
        else{
            return true;
        }
    }
    return false;
}

vector<FactPair> GoalSubsetSpace::get_current_goals()
{
    return get_goals(current_node);
}

GoalSubsets GoalSubsetSpace::generate_MUGS(){

    GoalSubsets mugs = GoalSubsets();

    for (GoalSpaceNode* node: generated){
        // node->print();
        if(node->isUnSolvable()){
            bool all_solvable = true;
            for (GoalSpaceNode* child : node->get_subsets()){
                all_solvable &= child->isSolvable();
            }
            if(all_solvable){
                mugs.add(node->get_goals());
            }
        }
    }

    mugs.minimize_non_minimal_subsets();
    return mugs;
}

GoalSubsets GoalSubsetSpace::generate_MSGS(){

    GoalSubsets msgs = GoalSubsets();

    for (GoalSpaceNode* node: generated){

        if(node->isSolvable()){
            bool all_unsolvable = true;
            for (GoalSpaceNode* child : node->get_supersets()){
                all_unsolvable &= child->isUnSolvable();
            }
            if(all_unsolvable){
                msgs.add(node->get_goals());
            }
        }
    }

    msgs.minimize_non_minimal_subsets();
    return msgs;
}


void GoalSubsetSpace::print(){
    GoalSubsets mugs =this->generate_MUGS();
    GoalSubsets msgs =this->generate_MSGS();

    cout << "*********************************"  << endl;
    cout << "Number of generated goal subsets: " << generated.size() << endl;
    // for(GoalSpaceNode* node : generated){
    //     node->get_goals().print();
    // }
    cout << "*********************************"  << endl;
    cout << "#hard goals: " << hard_goal_list.size() << endl;
    TaskProxy taskproxy = TaskProxy(*tasks::g_root_task.get());
    for(FactPair g : hard_goal_list){
        cout << "\t" << taskproxy.get_variables()[g.var].get_fact(g.value).get_name() << endl;
    }
    cout << "#soft goals: " << soft_goal_list.size() << endl;
    for(FactPair g : soft_goal_list){
        cout << "\t" << taskproxy.get_variables()[g.var].get_fact(g.value).get_name() << endl;
    }
    cout << "*********************************"  << endl;
    cout << "#MSGS: " << msgs.size() << endl;
    cout << "*********************************"  << endl;
    if(msgs.size() > 100){
        cout << "Too many msgs to print!" << endl;
    }
    else{
        msgs.print_subsets();
    }
    cout << "*********************************"  << endl;
    cout << "#MUGS: " << mugs.size() << endl;
    cout << "*********************************"  << endl;
    if(mugs.size() > 100){
        cout << "Too many mugs to print!" << endl;
    }
    else{
        mugs.print_subsets();
    }
    cout << "*********************************"  << endl;
    if(mugs.size() > 100){
        cout << "Too many mugs to print!" << endl;
    }
    else{
        mugs.print(soft_goal_fact_names);
    }
    cout << "*********************************"  << endl;
}

}
