#ifndef MSGS_EVALUATION_CONTEXT_H
#define MSGS_EVALUATION_CONTEXT_H

#include "../../evaluation_result.h"
#include "../../evaluator_cache.h"
#include "../../evaluation_context.h"
#include "../../operator_id.h"
#include "../../task_proxy.h"
#include "msgs_collection.h"

#include <unordered_map>

class Evaluator;
class SearchStatistics;

/*
  TODO: Now that we have an explicit EvaluationResult class, it's
  perhaps not such a great idea to duplicate all its access methods
  like "get_evaluator_value()" etc. on MSGSEvaluationContext. Might be a
  simpler interface to just give MSGSEvaluationContext an operator[]
  method or other simple way of accessing a given EvaluationResult
  and then use the methods of the result directly.
*/

/*
  MSGSEvaluationContext has two main purposes:

  1. It packages up the information that evaluators and open lists
     need in order to perform an evaluation: the state, the g value of
     the node, and whether it was reached by a preferred operator.

  2. It caches computed evaluator values and preferred operators for
     the current evaluation so that they do not need to be computed
     multiple times just because they appear in multiple contexts,
     and also so that we don't need to know a priori which evaluators
     need to be evaluated throughout the evaluation process.

     For example, our current implementation of A* search uses the
     evaluator value h at least three times: twice for its
     tie-breaking open list based on <g + h, h> and a third time for
     its "progress evaluator" that produces output whenever we reach a
     new best f value.
*/

class MSGSEvaluationContext : public EvaluationContext{

    MSGSCollection *current_msgs;
    int bound;

    MSGSEvaluationContext(
        const EvaluatorCache &cache, const State &state, int g_value,
        bool is_preferred, SearchStatistics *statistics, MSGSCollection* current_msgs, 
        int cost_bound, bool calculate_preferred);
public:
    /*
      Copy existing heuristic cache and use it to look up heuristic values.
      Used for example by lazy search.

      TODO: Can we reuse caches? Can we move them instead of copying them?
    */
    MSGSEvaluationContext(
        const MSGSEvaluationContext &other,
        int g_value, bool is_preferred, SearchStatistics *statistics,
        MSGSCollection* current_msgs, int cost_bound, bool calculate_preferred = false);
    /*
      Create new heuristic cache for caching heuristic values. Used for example
      by eager search.
    */
    MSGSEvaluationContext(
        const State &state, int g_value, bool is_preferred,
        SearchStatistics *statistics, MSGSCollection* current_msgs, int cost_bound, 
        bool calculate_preferred = false);
    /*
      Use the following constructor when you don't care about g values,
      preferredness (and statistics), e.g. when sampling states for heuristics.

      This constructor sets g_value to -1 and checks that neither get_g_value()
      nor is_preferred() are called for objects constructed with it.

      TODO: In the long term we might want to separate how path-dependent and
            path-independent evaluators are evaluated. This change would remove
            the need to store the g value and preferredness for evaluation
            contexts that don't need this information.
    */
    MSGSEvaluationContext(
        const State &state,
        SearchStatistics *statistics = nullptr, MSGSCollection* current_msgs = nullptr, bool calculate_preferred = false);

    const EvaluationResult &get_result(Evaluator *eval) override;

    bool is_evaluator_value_infinite(Evaluator *eval) override;
    int get_evaluator_value(Evaluator *eval) override;
    int get_evaluator_value_or_infinity(Evaluator *eval) override;

    MSGSCollection* get_msgs_collection() const;
    int get_cost_bound() const;
};

#endif
