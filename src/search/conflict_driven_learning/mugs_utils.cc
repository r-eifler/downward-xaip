#include "mugs_utils.h"

#include "../global_state.h"

#include <bitset>
#include <cassert>
#include <deque>
#include <iostream>
#include <sstream>
#include <string>

namespace conflict_driven_learning {
namespace mugs {

std::string
to_string(const subgoal_t& sg)
{
    std::ostringstream s;
    s << std::bitset<32>(sg);
    return s.str();
}

uint32_t
num_satisfied_goals(const subgoal_t& subgoal)
{
    // https://stackoverflow.com/questions/22081738/how-does-this-algorithm-to-count-the-number-of-set-bits-in-a-32-bit-integer-work
    uint32_t i = subgoal - ((subgoal >> 1) & 0x55555555);
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
    return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

bool
is_superset(const subgoal_t& super, const subgoal_t& sub)
{
    subgoal_t diff = super ^ sub;
    return (diff == 0) || (((diff & super) > 0) && ((diff & sub) == 0));
}

subgoal_t
get_subgoals(
    const GlobalState& state,
    const std::vector<std::pair<int, int>>& goal)
{
    subgoal_t res = 0U;
    for (unsigned i = 0; i < goal.size(); i++) {
        if (state[goal[i].first] == goal[i].second) {
            res = res | (1U << i);
        }
    }
    return res;
}

void
get_goal_facts(
    const subgoal_t& subgoal,
    const std::vector<std::pair<int, int>>& goal_facts,
    std::vector<std::pair<int, int>>& satisfied)
{
    for (unsigned i = 0; i < goal_facts.size(); i++) {
        if ((1U << i) & subgoal) {
            satisfied.push_back(goal_facts[i]);
        }
    }
}

SubgoalSet::SubgoalSet()
    : size_(0)
{
}

bool
SubgoalSet::contains_superset(const subgoal_t& sg) const
{
    return contains_superset2(sg).second;
}

bool
SubgoalSet::contains_subset(const subgoal_t& sg) const
{
    unsigned sgsize = num_satisfied_goals(sg);
    if (sgsize == 0) {
        return false;
    }
    for (unsigned i = 0; i < subgoals_.size(); i++) {
        const unsigned& size = subgoals_[i].first;
        const auto& subgoals = subgoals_[i].second;
        if (size >= sgsize) {
            return false;
        } else {
            for (auto it = subgoals.begin(); it != subgoals.end(); it++) {
                if (is_superset(sg, *it)) {
                    return true;
                }
            }
        }
    }
    return false;
}

std::pair<int, bool>
SubgoalSet::contains_superset2(const subgoal_t& sg) const
{
    using res_t = std::pair<int, bool>;
    unsigned sgsize = num_satisfied_goals(sg);
    if (sgsize == 0) {
        return res_t(-1, true);
    }
    for (int i = subgoals_.size() - 1; i >= 0; i--) {
        const unsigned& size = subgoals_[i].first;
        const auto& subgoals = subgoals_[i].second;
        if (size < sgsize) {
            return res_t(i, false);
        } else if (size == sgsize) {
            return res_t(i, subgoals.count(sg));
        } else {
            for (auto it = subgoals.begin(); it != subgoals.end(); it++) {
                if (is_superset(*it, sg)) {
                    return res_t(i, true);
                }
            }
        }
    }
    return res_t(-1, false);
}

bool
SubgoalSet::insert(
    const subgoal_t& sg,
    std::function<void(const subgoal_t&)> callback)
{
    std::pair<int, bool> contained = contains_superset2(sg);
    if (contained.second) {
        return false;
    }
    size_++;
    unsigned sgsize = num_satisfied_goals(sg);
    if (contained.first >= 0 && subgoals_[contained.first].first == sgsize) {
        subgoals_[contained.first].second.insert(sg);
        contained.first--;
    } else {
        std::pair<unsigned, std::unordered_set<subgoal_t>> new_sgs;
        new_sgs.first = sgsize;
        new_sgs.second.insert(sg);
        subgoals_.insert(
            subgoals_.begin() + (contained.first + 1), std::move(new_sgs));
    }
    for (; contained.first >= 0; contained.first--) {
        auto& sgs = subgoals_[contained.first].second;
        auto it = sgs.begin();
        while (it != sgs.end()) {
            if (is_superset(sg, *it)) {
                callback(*it);
                it = sgs.erase(it);
                size_--;
            } else {
                it++;
            }
        }
        if (sgs.empty()) {
            subgoals_.erase(subgoals_.begin() + contained.first);
        }
    }
#if 0
    std::cout << "Inserted " << to_string(sg) << " -> " << std::endl;
    for (unsigned i = 0; i < subgoals_.size(); i++) {
        std::cout << " [" << subgoals_[i].first << "]: {";
        bool sep = false;
        for (auto it = subgoals_[i].second.begin(); it != subgoals_[i].second.end(); it++) {
            std::cout << (sep ? ", " : "") << to_string(*it);
        }
        std::cout << "}" << std::endl;
    }
#endif

    return true;
}

std::unordered_set<subgoal_t>
SubgoalSet::get_minimal_extensions(unsigned width, const subgoal_t& filter)
    const
{
#if 1
    std::unordered_set<subgoal_t> ugs;
    std::deque<std::pair<unsigned, subgoal_t>> queue;
    for (unsigned i = 0; i < width; i++) {
        subgoal_t sg = 1U << i;
        if (contains_superset(sg)) {
            queue.emplace_back(i + 1, sg);
        } else if ((filter & sg) == filter) {
            ugs.insert(sg);
        }
    }
    while (!queue.empty()) {
        auto elem = queue.front();
        queue.pop_front();
        for (unsigned i = elem.first; i < width; i++) {
            subgoal_t sg = elem.second | (1U << i);
            if (sg != elem.second) {
                if (contains_superset(sg)) {
                    queue.emplace_back(i + 1, sg);
                } else if ((filter & sg) == filter) {
                    bool dominated = false;
                    for (auto it = ugs.begin(); it != ugs.end(); it++) {
                        if (is_superset(sg, *it)) {
                            dominated = true;
                            break;
                        }
                    }
                    if (!dominated)
                        ugs.insert(sg);
                }
            }
        }
    }
    return ugs;
#else
#if 0
    std::unordered_set<subgoal_t> ugs;
    std::unordered_set<subgoal_t> candidates;
    //all subset with one goal fact
    for(unsigned i = 0; i < width; i++){
        candidates.insert(1U << i);
    }

    while(candidates.size() > 0){
        //add goal facts until set is not solvable anymore
        auto it = candidates.begin();
        while( it != candidates.end()){
            subgoal_t gs = *it;
            it = candidates.erase(it);
            if(contains_superset(gs)){
                //create new candidates sets with one additional goal fact
                for(unsigned i = 0; i < width; i++){
                    if(((gs & (1U << i)) == 0)){
                        candidates.insert(gs | (1U << i));
                    }
                }
            }
            else if ((gs & filter) == filter) {
                ugs.insert(gs);               
            }
        }
    }

    /*
    cout << "----------------------------------"  << endl;
    cout << "Unsolvable goal subsets: " << endl;
    print_set(ugs);
    */
    return ugs;
#else
    std::unordered_set<subgoal_t> ugs;
    std::unordered_set<subgoal_t> candidates;
    std::unordered_set<subgoal_t> candidates1;

    for (unsigned i = 0; i < width; i++) {
        candidates1.insert(1U << i);
    }

    while (!candidates1.empty()) {
        assert(candidates.empty());
        candidates.swap(candidates1);
        for (auto it = candidates.begin(); it != candidates.end(); it++) {
            const subgoal_t& sg = *it;
            if (contains_superset(sg)) {
                for (unsigned i = 0; i < width; i++) {
                    subgoal_t succ = sg | (1U << i);
                    if (sg != succ) {
                        candidates1.insert(succ);
                    }
                }
            } else if ((filter & sg) == filter) {
                ugs.insert(sg);
            }
        }
        candidates.clear();
    }

    auto it1 = ugs.begin();
    while (it1 != ugs.end()) {
        auto cur = it1++;
        for (auto it2 = ugs.begin(); it2 != ugs.end(); it2++) {
            if (cur != it2 && is_superset(*cur, *it2)) {
                it1 = ugs.erase(cur);
                break;
            }
        }
    }

    return ugs;
#endif
#endif
}

SubgoalSet::const_iterator
SubgoalSet::begin() const
{
    return const_iterator(subgoals_.begin(), subgoals_.end());
}

SubgoalSet::const_iterator
SubgoalSet::end() const
{
    return const_iterator(subgoals_.end(), subgoals_.end());
}

size_t
SubgoalSet::size() const
{
    return size_;
}

} // namespace mugs
} // namespace conflict_driven_learning
