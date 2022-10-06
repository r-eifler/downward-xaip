#include "goal_subset_space.h"

#include "../task_proxy.h"
#include "../tasks/root_task.h"

#include "../utils/system.h"

#include "bitset"

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
        // if(!subset.is_empty())
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

GoalSubsetSpace::GoalSubsetSpace(GoalsProxy goals, bool all_soft_goals, bool weaken):
    weaken(weaken){

    TaskProxy taskproxy = TaskProxy(*tasks::g_root_task.get());

    if(all_soft_goals){
        for(uint i = 0; i < goals.size(); i++){
            soft_goal_list.push_back(goals[i].get_pair());
        }
    }
    else{
        for(uint i = 0; i < taskproxy.get_soft_goals().size(); i++){
            soft_goal_list.push_back(taskproxy.get_soft_goals()[i].get_pair());
        }
        for(uint i = 0; i < taskproxy.get_hard_goals().size(); i++){
            hard_goal_list.push_back(taskproxy.get_hard_goals()[i].get_pair());
        }
    }
    
    //sort goal facts according to theri variable id
    std::sort(soft_goal_list.begin(), soft_goal_list.end());

    std::bitset<64> init_goals = weaken ? (1U << soft_goal_list.size()) - 1 : 0;
    root = new GoalSpaceNode(GoalSubset(init_goals, soft_goal_list.size()));
    current_node = root;
    open_list.push_back(root); 
}

 void GoalSubsetSpace::current_goals_solved(bool solved, bool propagate){
        if (solved)
            current_node->solved(propagate);
        else
            current_node->not_solved(propagate);
    }


void GoalSubsetSpace::expand(){
    cout << "------------ EXPAND ------------" << endl;
    if ((weaken && current_node->isSolvable()) || (!weaken && current_node->isUnSolvable())){
        cout << "No expansion necessary" << endl;
        cout << "------------ EXPAND ------------" << endl;
        return;
    } 

    vector<GoalSpaceNode*> new_nodes = weaken ? current_node->weaken() : current_node->strengthen();

    
    cout << "generated:"  << endl;
    for (GoalSpaceNode* node : generated){
        node->get_goals().print();
    }
    cout << "children:"  << endl;
    //TODO implement a more efficient version
    //TODO Check whether do dublicate nodes are generated
    for (GoalSpaceNode* node : new_nodes){
        node->get_goals().print();
        auto old_node_it = generated.find(node);
        if(old_node_it != generated.end()){
            cout << "--> already exists"  << endl;
            current_node->addChild(*old_node_it);
            delete node;
        }
        else{
            cout << "--> new node"  << endl;
            current_node->addChild(node);
            generated.insert(node);
            open_list.push_front(node);
        }
    }
    cout << "------------ EXPAND ------------" << endl;
}

int GoalSubsetSpace::print_relation(){
    cout << "MUGS: " << endl;
    cout << "not yet implemented" << endl;
    return 0; // TODO root->print_relation(soft_goal_list);
}


vector<FactPair> GoalSubsetSpace::get_goals(const GoalSpaceNode* node) const
{
    vector<FactPair> current_goals;
    vector<FactPair> soft_goals = node->get_goals(soft_goal_list);
    current_goals.insert( current_goals.end(), hard_goal_list.begin(), hard_goal_list.end() );
    current_goals.insert( current_goals.end(), soft_goals.begin(), soft_goals.end() );
    return current_goals;
}

 GoalSpaceNode*  GoalSubsetSpace::get_next_node(){
    while(!open_list.empty()){
        cout << "size openlist: " << open_list.size() << endl;
        GoalSpaceNode* next_node = open_list.front();
        open_list.pop_front();
        if(!next_node->statusDefined())
            return next_node;
    }
    return NULL;
}

bool  GoalSubsetSpace::next_node(){
    current_node = get_next_node();
    return current_node != NULL;
}

vector<FactPair> GoalSubsetSpace::get_next_goals()
{
    return get_goals(current_node);
}


void GoalSubsetSpace::print(){
    cout << "*********************************"  << endl;
    cout << "Number of generated goal subsets: " << generated.size() << endl;
    cout << "Hard goals: " << endl;
    TaskProxy taskproxy = TaskProxy(*tasks::g_root_task.get());
    for(FactPair g : hard_goal_list){
        cout << taskproxy.get_variables()[g.var].get_fact(g.value).get_name() << endl;
    }
    cout << "Soft goals: " << endl;
    for(FactPair g : soft_goal_list){
        cout << taskproxy.get_variables()[g.var].get_fact(g.value).get_name() << endl;
    }
    cout << "*********************************"  << endl;
    int printed_nodes = print_relation();
    cout << "*********************************"  << endl;
    cout << "Number of minimal unsolvable goal subsets: " << printed_nodes << endl;
    cout << "*********************************"  << endl;
}

}
