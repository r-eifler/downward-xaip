#include "potential_function.h"
#include "potential_goals_heuristic.h"
#include "potential_optimizer.h"
#include "util.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../tasks/modified_goals_task.h"
#include "../lp/lp_solver.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"
#include "potential_max_heuristic.h"

#include <memory>
#include <vector>

using namespace std;

namespace potentials {

static void filter_dead_ends(PotentialOptimizer &optimizer, vector<State> &samples) {
    assert(!optimizer.potentials_are_bounded());
    vector<State> non_dead_end_samples;
    for (const State &sample : samples) {
        optimizer.optimize_for_state(sample);
        if (optimizer.has_optimal_solution())
            non_dead_end_samples.push_back(sample);
    }
    swap(samples, non_dead_end_samples);
}

static void optimize_for_samples(
    PotentialOptimizer &optimizer,
    int num_samples,
    utils::RandomNumberGenerator &rng) {
    vector<State> samples = sample_without_dead_end_detection(
        optimizer, num_samples, rng);
    if (!optimizer.potentials_are_bounded()) {
        filter_dead_ends(optimizer, samples);
    }
    optimizer.optimize_for_samples(samples);
}

static vector<unique_ptr<PotentialFunction>> create_goal_potential_functions(
    const Options &opts) {

    const shared_ptr<AbstractTask> task = opts.get<shared_ptr<AbstractTask>>("transform");
    TaskProxy task_proxy(*task);

    vector<unique_ptr<PotentialFunction>> functions;

    for(size_t i = 0; i < task_proxy.get_goals().size(); i++){
        FactPair selected_goal = task_proxy.get_goals()[i].get_pair();
        cout << "Gen potentials for " << selected_goal.var << " = " << selected_goal.value << endl;
        shared_ptr<AbstractTask> subtask = make_shared<extra_tasks::ModifiedGoalsTask>(task, std::vector<FactPair> {selected_goal});

        PotentialOptimizer optimizer(subtask, lp::LPSolverType::CPLEX, opts.get<double>("max_potential"));

        shared_ptr<utils::RandomNumberGenerator> rng(utils::parse_rng_from_options(opts));
        optimize_for_samples(optimizer, opts.get<int>("num_samples"), *rng);
        functions.push_back(optimizer.get_potential_function());
    }
    cout << "#functons: " << functions.size() << endl;
    return functions;
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Sample-based potential heuristics",
        "Maximum over multiple potential heuristics optimized for samples. " +
        get_admissible_potentials_reference());
    parser.add_option<int>(
        "num_samples",
        "Number of states to sample",
        "1000",
        Bounds("0", "infinity"));

    utils::add_rng_options(parser);
    prepare_parser_for_admissible_potentials(parser);

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;

    return make_shared<PotentialGoalsHeuristic>(
        opts, create_goal_potential_functions(opts));
}

static Plugin<Evaluator> _plugin("individual_goal_potentials", _parse);
}
