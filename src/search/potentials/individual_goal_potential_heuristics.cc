#include "potential_function.h"
#include "potential_goals_heuristic.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../tasks/modified_goals_task.h"
#include "../lp/lp_solver.h"

#include <memory>
#include <vector>

using namespace std;

namespace potentials {

static vector<unique_ptr<PotentialFunction>> create_goal_potential_functions(
    const Options &opts) {
    vector<unique_ptr<PotentialFunction>> functions;

    const shared_ptr<AbstractTask> task = opts.get<shared_ptr<AbstractTask>>("transform");
    TaskProxy task_proxy(*task);

    for(size_t i = 0; i < task_proxy.get_goals().size(); i++){
        FactPair selected_goal = task_proxy.get_goals()[i].get_pair();
        cout << "Gen potentials for " << selected_goal.var << " = " << selected_goal.value << endl;
        shared_ptr<AbstractTask> subtask = make_shared<extra_tasks::ModifiedGoalsTask>(task, std::vector<FactPair> {selected_goal});

        PotentialOptimizer optimizer(subtask, lp::LPSolverType::CPLEX, opts.get<double>("max_potential"));

        optimizer.optimize_for_state(task_proxy.get_initial_state());
        functions.push_back(optimizer.get_potential_function());

    }
    return functions;
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Sample-based potential heuristics",
        "Maximum over multiple potential heuristics optimized for samples. " +
        get_admissible_potentials_reference());

    prepare_parser_for_admissible_potentials(parser);

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;

    return make_shared<PotentialGoalsHeuristic>(
        opts, create_goal_potential_functions(opts));
}

static Plugin<Evaluator> _plugin("individual_goal_potentials", _parse);
}
