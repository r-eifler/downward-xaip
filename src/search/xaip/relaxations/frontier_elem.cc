#include "frontier_elem.h"


using namespace std;

FrontierElem::FrontierElem():
    parent(StateID::no_state), op(0){

}

FrontierElem::FrontierElem(StateID parent, OperatorID op):
    parent(parent), op(op){

}