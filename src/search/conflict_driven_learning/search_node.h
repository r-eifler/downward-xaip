#ifndef TARJAN_SEARCH_NODE_H
#define TARJAN_SEARCH_NODE_H

#include "search_node_info.h"
#include "../state_id.h"
#include "../operator_id.h"

namespace conflict_driven_learning
{
namespace tarjan_search
{

class SearchNode
{
private:
    SearchNodeInfo& info;
    StateID state_id;
public:
    SearchNode(SearchNodeInfo& info, StateID state_id);
    bool is_new() const;
    bool is_closed() const;
    bool is_dead_end() const;
    bool is_recognized_dead_end() const;
    bool is_onstack() const;
    unsigned get_index() const;
    unsigned get_lowlink() const;
    int get_h() const;
    StateID get_state_id() const;
    StateID get_parent_state_id() const;
    OperatorID get_operator() const;

    void popped_from_stack();
    void mark_dead_end();
    void mark_recognized_dead_end();
    void open_initial_state(int h);
    void open(const SearchNode& parent, const OperatorID& op, int h);
    bool update_h(int h);
    void close(unsigned index);
    bool update_lowlink(unsigned index);
};

}
}

#endif
