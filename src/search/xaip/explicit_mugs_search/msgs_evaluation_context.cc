#include "msgs_evaluation_context.h"
#include "../../evaluation_context.h"

#include "../../evaluation_result.h"
#include "../../evaluator.h"
#include "../../search_statistics.h"

#include <cassert>

using namespace std;

MSGSEvaluationContext::MSGSEvaluationContext(
    const EvaluatorCache &cache, const State &state, int g_value,
    bool is_preferred, SearchStatistics *statistics,
    MSGSCollection *current_msgs,
    bool calculate_preferred)
    :  EvaluationContext(cache, state, g_value, is_preferred, statistics, calculate_preferred),
    current_msgs(current_msgs){
}


MSGSEvaluationContext::MSGSEvaluationContext(
    const MSGSEvaluationContext &other, int g_value,
    bool is_preferred, SearchStatistics *statistics, MSGSCollection *current_msgs, bool calculate_preferred)
    : EvaluationContext(other.cache, other.state, g_value, is_preferred,
                        statistics, calculate_preferred),
    current_msgs(current_msgs) {
}

MSGSEvaluationContext::MSGSEvaluationContext(
    const State &state, int g_value, bool is_preferred,
    SearchStatistics *statistics, MSGSCollection *current_msgs, bool calculate_preferred)
    : EvaluationContext(EvaluatorCache(), state, g_value, is_preferred,
                        statistics, calculate_preferred) ,
    current_msgs(current_msgs){
}

MSGSEvaluationContext::MSGSEvaluationContext(
    const State &state,
    SearchStatistics *statistics, MSGSCollection *current_msgs, bool calculate_preferred)
    : EvaluationContext(EvaluatorCache(), state, INVALID, false,
                        statistics, calculate_preferred),
    current_msgs(current_msgs) {
}

const EvaluationResult &MSGSEvaluationContext::get_result(Evaluator *evaluator) {
    EvaluationResult &result = cache[evaluator];
    if (result.is_uninitialized()) {
        result = evaluator->compute_result(*this);
        if (statistics &&
            evaluator->is_used_for_counting_evaluations() &&
            result.get_count_evaluation()) {
            statistics->inc_evaluations();
        }
    }
    return result;
}

bool MSGSEvaluationContext::is_evaluator_value_infinite(Evaluator *eval) {
    return get_result(eval).is_infinite();
}

int MSGSEvaluationContext::get_evaluator_value(Evaluator *eval) {
    int h = get_result(eval).get_evaluator_value();
    assert(h != EvaluationResult::INFTY);
    return h;
}

int MSGSEvaluationContext::get_evaluator_value_or_infinity(Evaluator *eval) {
    return get_result(eval).get_evaluator_value();
}

MSGSCollection* MSGSEvaluationContext::get_msgs_collection() const{
    return current_msgs;
}


