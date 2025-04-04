#include "task_relaxation_tracker.h"
#include "../goal_subsets/output_handler.h"


using namespace std;

TaskRelaxationTracker::TaskRelaxationTracker(const std::shared_ptr<AbstractTask> task) {

    TaskProxy task_proxy = TaskProxy(*task);
    cout << "################# TASK RELAXATION TRACKER ##############" << endl;

    for (RelaxedTaskDefinition tc : task_proxy.get_relaxed_task().get_relaxed_task_definitions()) {
        RelaxedTask* releaxed_task = new RelaxedTask(task, tc.id, tc.name, tc.init, tc.applicable_actions);
        relaxed_tasks.push_back(releaxed_task);
    }
    cout << "#relaxed tasks: " << relaxed_tasks.size() << endl;
    for (RelaxedTaskDefinition tc : task_proxy.get_relaxed_task().get_relaxed_task_definitions()){
        assert(relaxed_tasks.size() > tc.id);
        RelaxedTask* rt = relaxed_tasks[tc.id];
        for (int t_id : tc.lower_cover){
            assert(relaxed_tasks.size() > t_id);
            rt->add_to_lover_cover(relaxed_tasks[t_id]);
        }
        for (int t_id : tc.upper_cover){
            assert(relaxed_tasks.size() > t_id);
            rt->add_to_upper_cover(relaxed_tasks[t_id]);
        }
    }
}

bool TaskRelaxationTracker::has_next_relaxed_task(){
    int num_not_solvable = 0;
    for (int i = current_index + 1; i < (int) relaxed_tasks.size(); i++){
        if (! relaxed_tasks[i]->get_solvable())
            num_not_solvable++;
    }
//    cout << "Num unsolved tasks: " << num_not_solvable << endl;
    return num_not_solvable > 0;
}

RelaxedTask* TaskRelaxationTracker::next_relaxed_task(){
    current_index++;
    for (;current_index < (int) relaxed_tasks.size(); current_index++){
        if (! relaxed_tasks[current_index]->get_solvable())
            return relaxed_tasks[current_index];
    }
    return NULL;
}

RelaxedTask* TaskRelaxationTracker::current_relaxed_task(){
    return relaxed_tasks[current_index];
}

void TaskRelaxationTracker::results_to_file(){
    OutputHandler output_handler = OutputHandler("relaxation_mugs.json", true);
    for(RelaxedTask* task: relaxed_tasks){
        output_handler.add_collection(task->get_name(), task->get_mugs_string(), task->get_msgs_string());
    }
    output_handler.output();
}

