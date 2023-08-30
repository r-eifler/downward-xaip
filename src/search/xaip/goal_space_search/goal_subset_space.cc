#include "goal_subset_space.h"

#include "../task_proxy.h"
#include "../tasks/root_task.h"

#include "../utils/system.h"

#include <boost/dynamic_bitset.hpp>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/strong_components.hpp>
#include <iostream>
#include <vector>

using boost::make_iterator_range;

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

vector<GoalSpaceNode*> GoalSpaceNode::weaken(std::unordered_map<int, goalsubset::GoalSubset>* next_weaker) const {
    vector<GoalSubset> new_subsets;

    for (long unsigned int i = 0; i < goals.size(); i++){
        if(goals.contains(i)){
            GoalSubset weaker_subset = goals.clone();
            weaker_subset.remove(i);
            new_subsets.push_back(GoalSubset(weaker_subset));
            GoalSubset weaker_goals = (*next_weaker)[i];
            if (! weaker_goals.is_empty()){
                for (long unsigned int j = 0; j < weaker_goals.size(); j++){
                    if(weaker_goals.contains(j)){
                        GoalSubset weaker_subset = goals.clone();
                        weaker_subset.remove(i);
                        weaker_subset.add(j);
                        new_subsets.push_back(GoalSubset(weaker_subset));
                    }
                }
            }
        }
    }

    
    vector<GoalSpaceNode*> new_nodes;
    for(GoalSubset subset : new_subsets){
        new_nodes.push_back(new GoalSpaceNode(subset));
    }
    return new_nodes;
}

vector<GoalSpaceNode*> GoalSpaceNode::strengthen(std::unordered_map<int, goalsubset::GoalSubset>* next_stronger, 
    goalsubset::GoalSubset weakest, std::unordered_map<int, goalsubset::GoalSubset*>* connected_soft_goals) const{
    vector<GoalSubset> new_subsets;

    for (long unsigned int i = 0; i < goals.size(); i++){
        if(!goals.contains(i)){
            if (weakest.contains(i) && ((*connected_soft_goals)[i])->set_intersection(goals).is_empty()){
                GoalSubset stronger_subset = goals.clone();
                stronger_subset.add(i);
                new_subsets.push_back(GoalSubset(stronger_subset));
            }
        }
        else{
            GoalSubset stronger_goals = (*next_stronger)[i];
            if (! stronger_goals.is_empty()){
                for (long unsigned int j = 0; j < stronger_goals.size(); j++){
                    if(stronger_goals.contains(j)){
                        GoalSubset stronger_subset = goals.clone();
                        stronger_subset.remove(i);
                        stronger_subset.add(j);
                        new_subsets.push_back(GoalSubset(stronger_subset));
                    }
                }
            }
        }
    }

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

    // init goal list
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
        cout << soft_goal_fact_names[i] << endl;
    }

    // init soft goal graph
   this->init_soft_goal_relations();


    // gen first nodes
    this->init_root_node();
    
}

void GoalSubsetSpace::init_soft_goal_relations(){

    typedef boost::adjacency_list<boost::vecS, boost::listS, boost::directedS,
            boost::property<boost::vertex_index_t, int>> Graph;

    Graph soft_goal_graph(soft_goal_list.size());
    auto idmap = get(boost::vertex_index, soft_goal_graph);
    {
        // initialize idmap
        int id = 0;
        for (auto& v : make_iterator_range(vertices(soft_goal_graph)))
            idmap[v] = id++;
    }

    for(uint j = 0; j < soft_goal_list.size(); j++){
        next_weaker[j] = GoalSubset(soft_goal_list.size());
        next_stronger[j] = GoalSubset(soft_goal_list.size());
    }

    TaskProxy task_proxy = TaskProxy(*tasks::g_root_task.get());
    SoftGoalGraphProxy soft_goal_graph_proxy = task_proxy.get_soft_goal_graph();
    for (uint i = 0; i < soft_goal_graph_proxy.size(); i++){
        FactProxy source_proxy = soft_goal_graph_proxy[i].first;
        FactProxy target_proxy = soft_goal_graph_proxy[i].second;
        uint source_index = 0;
        uint target_index = 0;
        for(uint j = 0; j < soft_goal_list.size(); j++){
            if(soft_goal_list[j].value == source_proxy.get_pair().value && soft_goal_list[j].var == source_proxy.get_pair().var){
                source_index = j;
            }
            if(soft_goal_list[j].value == target_proxy.get_pair().value && soft_goal_list[j].var == target_proxy.get_pair().var){
                target_index = j;
            }
        }
        next_weaker[source_index].add(target_index);
        next_stronger[target_index].add(source_index);
        boost::add_edge(vertex(source_index, soft_goal_graph), vertex(target_index, soft_goal_graph), soft_goal_graph);
        boost::add_edge(vertex(target_index, soft_goal_graph), vertex(source_index, soft_goal_graph), soft_goal_graph);
    }

    // get weakest and strongest soft goals 
    weakest = GoalSubset(soft_goal_list.size());
    strongest = GoalSubset(soft_goal_list.size());
    for(uint j = 0; j < soft_goal_list.size(); j++){
        if (next_weaker[j].is_empty()){
            weakest.add(j);
        }
        if (next_stronger[j].is_empty()){
            strongest.add(j);
        }
    }

    // strongly connected componenets
    std::vector<int> c(num_vertices(soft_goal_graph));

    int num = strong_components(
        soft_goal_graph, make_iterator_property_map(c.begin(), idmap, c[0]));

    std::unordered_map<int, goalsubset::GoalSubset*> strongly_connected_componenets;
    for (int i = 0; i < num; i++){
        strongly_connected_componenets[i] = new GoalSubset(soft_goal_list.size());
    }
    for (uint i = 0; i < c.size(); i++){
        strongly_connected_componenets[c[i]]->add(i);
        connected_soft_goals[i] = strongly_connected_componenets[c[i]];
    }
}

void GoalSubsetSpace::init_root_node(){
    if (weaken){
        GoalSpaceNode* root = new GoalSpaceNode(strongest);
        current_node = root;
        open_list.push_back(root); 
        generated.insert(root);
    }
    else {
        for (GoalSubset gs : weakest.singelten_subsets()){
            gs.print();
            GoalSpaceNode* node = new GoalSpaceNode(gs);
            current_node = node;
            open_list.push_back(node); 
            generated.insert(node);
        }
    }

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

    vector<GoalSpaceNode*> new_nodes = weaken ? current_node->weaken(&next_weaker) : current_node->strengthen(&next_stronger, weakest, &connected_soft_goals);
    
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
