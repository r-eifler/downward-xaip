#ifndef MUGS_UTILS_H
#define MUGS_UTILS_H

#include <cassert>
#include <functional>
#include <iostream>
#include <iterator>
#include <string>
#include <unordered_set>
#include <vector>
#include <stdint.h>

class State;

namespace conflict_driven_learning {
namespace mugs {

using subgoal_t = uint32_t;

std::string to_string(const subgoal_t& sg);

extern unsigned num_satisfied_goals(const subgoal_t& subgoal);

extern bool is_superset(const subgoal_t& super, const subgoal_t& sub);

extern subgoal_t get_subgoals(
    const State& state,
    const std::vector<std::pair<int, int>>& goal);

extern void get_goal_facts(
    const subgoal_t& subgoal,
    const std::vector<std::pair<int, int>>& goal_facts,
    std::vector<std::pair<int, int>>& satisfied);

template<typename Iterator>
void print_set(Iterator begin, Iterator end);

template<typename Iterator>
void print_set(
    Iterator begin,
    Iterator end,
    const std::vector<std::string>& fact_names,
    const subgoal_t& hide = 0);

class SubgoalSet {
private:
    using internal_storage_t =
        std::vector<std::pair<unsigned, std::unordered_set<subgoal_t>>>;

public:
    class const_iterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = subgoal_t;
        using difference_type = int;
        using pointer = const value_type*;
        using reference = const value_type&;

        const_iterator& operator++()
        {
            it_++;
            if (it_ == partition_it_->second.end()) {
                partition_it_++;
                if (partition_it_ != partition_end_) {
                    it_ = partition_it_->second.begin();
                }
            }
            return *this;
        }

        const_iterator operator++(int)
        {
            const_iterator res(*this);
            ++(*this);
            return res;
        }

        bool operator==(const const_iterator& other) const
        {
            if (partition_it_ != other.partition_it_) {
                return false;
            }
            assert(partition_end_ == other.partition_end_);
            if (partition_it_ == partition_end_) {
                return true;
            }
            return it_ == other.it_;
        }

        bool operator!=(const const_iterator& other) const
        {
            return !(*this == other);
        }

        reference operator*() const { return *it_; }

    private:
        friend class SubgoalSet;
        explicit const_iterator(
            internal_storage_t::const_iterator begin,
            internal_storage_t::const_iterator end)
            : partition_it_(begin)
            , partition_end_(end)
        {
            if (begin != end) {
                it_ = begin->second.begin();
            }
        }
        internal_storage_t::const_iterator partition_it_;
        const internal_storage_t::const_iterator partition_end_;
        std::unordered_set<subgoal_t>::const_iterator it_;
    };

    SubgoalSet();
    SubgoalSet(const SubgoalSet&) = default;
    SubgoalSet(SubgoalSet&&) = default;
    ~SubgoalSet() = default;

    SubgoalSet& operator=(const SubgoalSet&) = default;
    SubgoalSet& operator=(SubgoalSet&&) = default;

    bool contains_subset(const subgoal_t& sg) const;
    bool contains_superset(const subgoal_t& sg) const;
    bool insert(
        const subgoal_t& sg,
        std::function<void(const subgoal_t&)> callback = [](const subgoal_t&) {
        });
    std::unordered_set<subgoal_t> get_minimal_extensions(
        unsigned width,
        const subgoal_t& filter = (1U >> 1)) const;
    const_iterator begin() const;
    const_iterator end() const;
    size_t size() const;

private:
    std::pair<int, bool> contains_superset2(const subgoal_t& sg) const;

    internal_storage_t subgoals_;
    size_t size_;
};

} // namespace mugs
} // namespace conflict_driven_learning

namespace conflict_driven_learning {
namespace mugs {

template<typename Iterator>
void
print_set(Iterator begin, Iterator end)
{
    while (begin != end) {
        std::cout << to_string(*begin) << std::endl;
        ++begin;
    }
}

template<typename Iterator>
void
print_set(
    Iterator begin,
    Iterator end,
    const std::vector<std::string>& fact_names,
    const subgoal_t& hide)
{
    // assert(fact_names.size() <= sizeof(subgoal_t) * CHAR_BIT);
    while (begin != end) {
        subgoal_t sg = *begin;
        bool sep = false;
        for (unsigned i = 0; i < fact_names.size(); i++) {
            if (((1U << i) & sg) && !(((1U << i) & hide))) {
                std::cout << (sep ? "|" : "") << fact_names[i];
                sep = true;
            }
        }
        // std::cout << " <" << to_string(sg) << ">"
        std::cout << std::endl;
        ++begin;
    }
}

} // namespace mugs
} // namespace conflict_driven_learning

#endif
