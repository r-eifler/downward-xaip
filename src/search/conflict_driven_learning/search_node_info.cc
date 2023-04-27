#include "search_node_info.h"

namespace conflict_driven_learning
{
namespace tarjan_search
{
int SearchNodeInfo::NEW = -2;
int SearchNodeInfo::DEAD_END = -1;
int SearchNodeInfo::RECOGNIZED_DEAD_END = -3;
unsigned SearchNodeInfo::UNDEFINED = -1;
}
}

