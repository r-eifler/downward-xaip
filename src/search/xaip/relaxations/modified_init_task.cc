#include "modified_init_task.h"

using namespace std;

namespace extra_tasks {
ModifiedInitTask::ModifiedInitTask(
    const shared_ptr<AbstractTask> &parent, vector<FactPair> &&change_init)
    : DelegatingTask(parent) {
//            cout << "------ ModifiedInitTask ------ " << endl;
          init = parent->get_initial_state_values();
          for(FactPair fp : change_init){
//              cout << fp << endl;
              init[fp.var] = fp.value;
          }
}


std::vector<int> ModifiedInitTask::get_initial_state_values() const {
    return init;
}
}
