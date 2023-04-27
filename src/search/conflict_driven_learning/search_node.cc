
#include "search_node.h"

#include <cassert>

namespace conflict_driven_learning
{
namespace tarjan_search
{

SearchNode::SearchNode(SearchNodeInfo& info, StateID state_id)
    : info(info), state_id(state_id)
{
}

bool SearchNode::is_new() const
{
    return info.h == SearchNodeInfo::NEW;
}

bool SearchNode::is_closed() const
{
    return info.index != SearchNodeInfo::UNDEFINED;
}

bool SearchNode::is_onstack() const
{
    return info.onstack;
}

bool SearchNode::is_dead_end() const
{
    return info.h == SearchNodeInfo::DEAD_END || info.h == SearchNodeInfo::RECOGNIZED_DEAD_END;
}

bool SearchNode::is_recognized_dead_end() const
{
    return info.h == SearchNodeInfo::RECOGNIZED_DEAD_END;
}

unsigned SearchNode::get_index() const
{
    return info.index;
}

unsigned SearchNode::get_lowlink() const
{
    return info.lowlink;
}

int SearchNode::get_h() const
{
    return info.h;
}

StateID SearchNode::get_state_id() const
{
    return state_id;
}

StateID SearchNode::get_parent_state_id() const
{
    return info.parent;
}

OperatorID SearchNode::get_operator() const
{
    return info.op;
}

void SearchNode::mark_dead_end()
{
    assert(info.h != SearchNodeInfo::RECOGNIZED_DEAD_END);
    info.h = SearchNodeInfo::DEAD_END;
}

void SearchNode::mark_recognized_dead_end()
{
    info.h = SearchNodeInfo::RECOGNIZED_DEAD_END;
}

void SearchNode::popped_from_stack()
{
    info.onstack = 0;
}

void SearchNode::close(unsigned index)
{
    assert(info.index == info.lowlink && info.index == SearchNodeInfo::UNDEFINED);
    info.onstack = 1;
    info.index = index;
    info.lowlink = index;
}

void SearchNode::open_initial_state(int h)
{
    info.h = h;
}

bool SearchNode::update_h(int h)
{
    if (h > info.h) {
        info.h = h;
        return true;
    }
    return false;
}

void SearchNode::open(
    const SearchNode& parent,
    const OperatorID& op,
    int h)
{
    info.parent = parent.get_state_id();
    info.op = op;
    info.h = h;
}

bool SearchNode::update_lowlink(unsigned index)
{
    if (index != SearchNodeInfo::UNDEFINED && index < info.lowlink) {
        info.lowlink = index;
        return true;
    }
    return false;
}

}
}
