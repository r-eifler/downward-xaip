#ifndef TARJAN_SEARCH_SPACE_H
#define TARJAN_SEARCH_SPACE_H

#include "../state_registry.h"
#include "../per_state_information.h"
#include "../operator_id.h"

#include <vector>
#include <algorithm>

namespace conflict_driven_learning {

template<typename Info, typename Node>
class SearchSpaceBase
{
private:
    const StateRegistry* m_state_registry;
    PerStateInformation<Info> m_data;
public:
    SearchSpaceBase(const StateRegistry* state_registry);
    virtual ~SearchSpaceBase() = default;
    Node operator[](const State& state);
    void trace_path(Node node, std::vector<OperatorID>& path);
};

template<typename Info, typename Node>
SearchSpaceBase<Info, Node>::SearchSpaceBase(
    const StateRegistry* state_registry)
    : m_state_registry(state_registry)
{}

template<typename Info, typename Node>
Node
SearchSpaceBase<Info, Node>::operator[](
    const State& state)
{
    return Node(m_data[state], state.get_id());
}

template<typename Info, typename Node>
void
SearchSpaceBase<Info, Node>::trace_path(
        Node node,
        std::vector<OperatorID>& path)
{
    if (node.get_operator() != OperatorID::no_operator) {
        path.push_back(node.get_operator());
        trace_path(
                operator[](m_state_registry->lookup_state(node.get_parent_state_id())),
                path);
    } else {
        std::reverse(path.begin(), path.end());
    }
}

}

#endif
