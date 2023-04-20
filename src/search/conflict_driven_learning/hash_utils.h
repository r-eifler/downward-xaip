#ifndef CDL_HASH_UTILS_H
#define CDL_HASH_UTILS_H

#include "../utils/hash.h"
#include "../algorithms/segmented_vector.h"

#include <vector>
#include <algorithm>

namespace conflict_driven_learning {
namespace hash_utils {

template<typename Collection>
class CollectionIdHash {
public:
    CollectionIdHash(const Collection& collection) : m_cref(collection) {}

    std::size_t
    operator()(unsigned ref) const
    {
        const auto& seq = m_cref[ref];
        return utils::hash_sequence(seq, seq.size());
    }

private:
    const Collection& m_cref;
};

template<typename T>
using SegVecIdHash = CollectionIdHash<segmented_vector::SegmentedVector<std::vector<T> > >;

template<typename T>
using VecIdHash = CollectionIdHash<std::vector<std::vector<T> > >;


template<typename Collection>
class CollectionIdEqual {
public:
    CollectionIdEqual(const Collection& collection) : m_cref(collection) {}

    bool
    operator()(unsigned x, unsigned y) const
    {
        const auto& seq_x = m_cref[x];
        const auto& seq_y = m_cref[y];
        if (seq_x.size() != seq_y.size()) {
            return false;
        }
        auto it_x = seq_x.begin();
        auto it_y = seq_y.begin();
        auto end = seq_x.end();
        while (it_x != end) {
            if (*it_x != *it_y) {
                return false;
            }
            it_x++;
            it_y++;
        }
        return true;
    }

private:
    const Collection& m_cref;
};

template<typename T>
using SegVecIdEqual = CollectionIdEqual<segmented_vector::SegmentedVector<std::vector<T> > >;

template<typename T>
using VecIdEqual = CollectionIdEqual<std::vector<std::vector<T> > >;

}
}

#endif
