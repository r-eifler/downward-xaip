#ifndef TARJAN_SEARCH_NODE_INFO_H
#define TARJAN_SEARCH_NODE_INFO_H

#include "../state_id.h"
#include "../operator_id.h"

namespace conflict_driven_learning
{
namespace tarjan_search
{
struct SearchNodeInfo {
    static int NEW;
    static int DEAD_END;
    static int RECOGNIZED_DEAD_END;
    static unsigned UNDEFINED;
    int h : 31;
    unsigned onstack : 1;
    unsigned index;
    unsigned lowlink;
    StateID parent;
    OperatorID op;
    SearchNodeInfo()
        : h(NEW),
          onstack(0),
          index(UNDEFINED),
          lowlink(UNDEFINED),
          parent(StateID::no_state),
          op(OperatorID::no_operator)
    {}
};
}
}

#endif
