#ifndef FAST_DOWNWARD_DRONTIER_ELEM_H
#define FAST_DOWNWARD_DRONTIER_ELEM_H

#include "../../open_list.h"

#include <string>
#include <iostream>


using namespace std;

class FrontierElem {

public:
    
    StateID parent;
    OperatorID op;

    FrontierElem();
    FrontierElem(StateID parent, OperatorID op);

    bool operator==(const FrontierElem &other) const {
        return parent == other.parent && op == other.op;
    }

    bool operator!=(const FrontierElem &other) const {
        return !(*this == other);
    }
    
};

struct HashFrontierElem {
  std::size_t operator()(const FrontierElem& o) const {
      return o.parent.hash() ^ o.op.hash() ;
  }
};



#endif 
