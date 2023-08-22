#ifndef FORMULA_H
#define FORMULA_H

#include "set_utils.h"

#include <vector>
#include <utility>
#include <algorithm>
#include <cstdio>
#include <string>
#include <iostream>

namespace conflict_driven_learning
{

class CounterBasedFormula
{
#ifndef NDEBUG
    bool d_initialized;
#endif
    std::vector<std::vector<unsigned> > m_rel;
    std::vector<unsigned> m_sizes;
    std::vector<unsigned> m_subset;
public:
    CounterBasedFormula()
    {
#ifndef NDEBUG
        d_initialized = false;
#endif
    }
    void set_num_keys(const unsigned& num)
    {
#ifndef NDEBUG
        d_initialized = true;
#endif
        m_rel.resize(num);
    }
    bool insert_element(unsigned id, const std::vector<unsigned> &, unsigned elem,
                        bool)
    {
        assert(d_initialized);
        m_sizes[id]++;
        set_utils::insert(m_rel[elem], id);
        return true;
    }
    bool delete_element(unsigned id, const std::vector<unsigned> &, unsigned elem)
    {
        assert(d_initialized);
        m_sizes[id]--;
        set_utils::remove(m_rel[elem], id);
        return true;
    }
    void erase(unsigned id, const std::vector<unsigned> &set)
    {
        assert(d_initialized);
        for (const unsigned &x : set) {
            set_utils::remove(m_rel[x], id);
        }
    }
    template<typename Callback>
    bool forall_subsets(const std::vector<unsigned> &set, const Callback &callback)
    {
        assert(d_initialized);
        std::fill(m_subset.begin(), m_subset.end(), 0);
        for (const unsigned &x : set) {
            for (const unsigned &y : m_rel[x]) {
                if (++m_subset[y] == m_sizes[y]) {
                    if (callback(y)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    template<typename Callback>
    void forall_supersets(const std::vector<unsigned> &set, const Callback &callback)
    {
        assert(d_initialized);
        std::fill(m_subset.begin(), m_subset.end(), 0);
        for (const unsigned &x : set) {
            for (const unsigned &y : m_rel[x]) {
                if (++m_subset[y] == set.size()) {
                    callback(y);
                }
            }
        }
    }
    unsigned insert(const std::vector<unsigned> &set)
    {
        assert(d_initialized);
        unsigned id = m_sizes.size();
        m_sizes.push_back(set.size());
        m_subset.push_back(0);
        for (const unsigned &x : set) {
            m_rel[x].push_back(id);
        }
        return id;
    }
    bool contains_subset_of(const std::vector<unsigned> &set)
    {
        assert(d_initialized);
        std::fill(m_subset.begin(), m_subset.end(), 0);
        for (const unsigned &x : set) {
            for (const unsigned &y : m_rel[x]) {
                if (++m_subset[y] == m_sizes[y]) {
                    return true;
                }
            }
        }
        return false;
    }
    bool is_cut_by(const std::vector<unsigned> &set)
    {
        assert(d_initialized);
        std::fill(m_subset.begin(), m_subset.end(), 0);
        unsigned left = m_sizes.size();
        for (unsigned i = 0; left > 0 && i < set.size(); i++) {
            const std::vector<unsigned> &subsets = m_rel[set[i]];
            for (unsigned j = 0; left > 0 && j < subsets.size(); j++) {
                const unsigned &id = subsets[j];
                if (!m_subset[id]) {
                    m_subset[id] = 1;
                    left--;
                }
            }
        }
        return left == 0;
    }
    size_t size() const
    {
        assert(d_initialized);
        return m_sizes.size();
    }
    void clear()
    {
        std::vector<std::vector<unsigned> > new_rel(m_rel.size());
        m_rel.swap(new_rel);
        m_sizes.clear();
        m_subset.clear();
    }
    // void statistics(const std::string &name = "Formula") const
    // {
    //     printf("%s size: %zu\n", name, m_sizes.size());
    // }
};


template<typename K>
class UBTreeFormula
{
public:
    struct Node;
    struct Node : public std::vector<std::pair<K, Node *> > {};
protected:
    Node *m_root;

    void clear_node(Node *node) const;
    Node *lookup_successor(Node *node, const K &key, unsigned &k) const;
    Node *lookup_successor(Node *node, const K &key) const;
    const Node *lookup_successor(const Node *node, const K &key) const;

    Node *insert(Node *node, const std::vector<K> &set, unsigned i) const;
    Node *force_insert(Node *node, const std::vector<K> &set, unsigned i) const;
    Node *insert(Node *node, const std::vector<K> &set, unsigned i,
                 bool &inserted) const;
    bool erase(Node *node, const std::vector<K> &set, unsigned i) const;
    bool erase(Node *node, const std::vector<K> &set, unsigned i,
               bool &found) const;
    bool remove_all_supersets_of(Node *node,
                                 const std::vector<K> &set,
                                 unsigned i) const;
    bool contains_subset_of(const Node *node,
                            const std::vector<K> &set,
                            unsigned i) const;
    bool is_cut_by(const Node *node, const std::vector<K> &set, unsigned i) const;
    size_t count_markers(const Node *node) const;
    void print(const Node *node, std::vector<K> &subset) const;
public:
    UBTreeFormula();
    ~UBTreeFormula();

    void set_num_keys(const unsigned&) {}

    bool contains_subset_of(const std::vector<K> &set) const;
    bool contains_set(const std::vector<K> &set) const;
    bool is_cut_by(const std::vector<K> &set) const;

    std::pair<Node *, bool> insert(const std::vector<K> &set);
    void erase(unsigned, const std::vector<K> &set);
    void remove_all_supersets_of(const std::vector<K> &set);

    unsigned delete_element(unsigned, const std::vector<K> &set, K elem);
    bool insert_element(unsigned,
                        const std::vector<K> &set,
                        K elem,
                        unsigned status);

    size_t size() const;

    void print() const;
    void clear();
};

template<typename K>
UBTreeFormula<K>::UBTreeFormula()
{
    m_root = new Node;
}

template<typename K>
UBTreeFormula<K>::~UBTreeFormula()
{
    clear_node(m_root);
    delete(m_root);
}

template<typename K>
void UBTreeFormula<K>::clear()
{
    clear_node(m_root);
}

template<typename K>
void UBTreeFormula<K>::clear_node(Node *node) const
{
    for (unsigned i = 0; i < node->size(); i++) {
        clear_node(node->at(i).second);
        delete(node->at(i).second);
    }
    node->clear();
}

template<typename K>
typename UBTreeFormula<K>::Node *UBTreeFormula<K>::lookup_successor(
    Node *node, const K &key, unsigned &k) const
{
    for (k = 0; k < node->size(); k++) {
        if (node->at(k).first == key) {
            return node->at(k).second;
        }
    }
    return NULL;
}

template<typename K>
typename UBTreeFormula<K>::Node *UBTreeFormula<K>::lookup_successor(
    Node *node, const K &key) const
{
    unsigned k;
    return lookup_successor(node, key, k);
}

template<typename K>
const typename UBTreeFormula<K>::Node *UBTreeFormula<K>::lookup_successor(
    const Node *node, const K &key) const
{
    for (unsigned k = 0; k < node->size(); k++) {
        if (node->at(k).first == key) {
            return node->at(k).second;
        }
    }
    return NULL;
}

template<typename K>
typename UBTreeFormula<K>::Node *UBTreeFormula<K>::insert(
    Node *node,
    const std::vector<K> &set,
    unsigned i) const
{
    Node *succ = NULL;
    for (; i < set.size(); i++) {
        succ = lookup_successor(node, set[i]);
        if (succ == NULL) {
            succ = new Node;
            node->emplace_back(set[i], succ);
        } else if (succ->empty()) {
            node = succ;
            break;
        }
        node = succ;
    }
    clear_node(node);
    return node;
}

template<typename K>
typename UBTreeFormula<K>::Node *UBTreeFormula<K>::insert(
    Node *node,
    const std::vector<K> &set,
    unsigned i,
    bool &inserted) const
{
    Node *succ = NULL;
    inserted = true;
    for (; i < set.size(); i++) {
        succ = lookup_successor(node, set[i]);
        if (succ == NULL) {
            succ = new Node;
            node->emplace_back(set[i], succ);
        } else if (succ->empty()) {
            inserted = false;
            node = succ;
            break;
        }
        node = succ;
    }
    clear_node(node);
    return node;
}

template<typename K>
typename UBTreeFormula<K>::Node *UBTreeFormula<K>::force_insert(
    Node *node,
    const std::vector<K> &set,
    unsigned i) const
{
    Node *succ = NULL;
    for (; i < set.size(); i++) {
        succ = lookup_successor(node, set[i]);
        if (succ == NULL) {
            succ = new Node;
            node->emplace_back(set[i], succ);
        }
        node = succ;
    }
    clear_node(node);
    return node;
}

template<typename K>
bool UBTreeFormula<K>::erase(Node *node,
                             const std::vector<K> &set,
                             unsigned i) const
{
    if (i == set.size()) {
        assert(node->empty());
        return true;
    }

    if (node->empty()) {
        return false;
    }

    unsigned k;
    Node *succ = lookup_successor(node, set[i], k);
    if (succ != NULL && erase(succ, set, i + 1)) {
        delete(succ);
        node->erase(node->begin() + k);
    }

    return node->empty();
}

template<typename K>
bool UBTreeFormula<K>::erase(Node *node,
                             const std::vector<K> &set,
                             unsigned i,
                             bool &found) const
{
    if (i == set.size()) {
        if (!node->empty()) {
            return false;
        } else {
            found = true;
            return true;
        }
    }

    if (node->empty()) {
        return false;
    }

    unsigned k;
    Node *succ = lookup_successor(node, set[i], k);
    if (succ != NULL && erase(succ, set, i + 1, found)) {
        delete(succ);
        node->erase(node->begin() + k);
    }

    return node->empty();
}

template<typename K>
bool UBTreeFormula<K>::remove_all_supersets_of(
    Node *node,
    const std::vector<K> &set,
    unsigned i) const
{
    if (i == set.size()) {
        clear_node(node);
        return true;
    }
    if (node->empty()) {
        return false;
    }
    unsigned k = node->size() - 1;
    while (k < node->size()) {
        if (node->at(k).first == set[i]) {
            if (remove_all_supersets_of(node->at(k).second, set, i + 1)) {
                delete(node->at(k).second);
                node->erase(node->begin() + k);
            }
        } else if (node->at(k).first > set[i]) {
            if (remove_all_supersets_of(node->at(k).second, set, i)) {
                delete(node->at(k).second);
                node->erase(node->begin() + k);
            }
        }
        k--;
    }
    return node->empty();
}


template<typename K>
bool UBTreeFormula<K>::contains_subset_of(
    const Node *node,
    const std::vector<K> &set,
    unsigned i) const
{
    if (node->empty()) {
        return true;
    }
    if (i == set.size()) {
        return false;
    }
    unsigned j;
    for (unsigned k = 0; k < node->size(); k++) {
        j = i;
        while (j < set.size() && set[j] < node->at(k).first) {
            j++;
        }
        if (j < set.size()
            && set[j] == node->at(k).first
            && contains_subset_of(node->at(k).second, set, j + 1)) {
            return true;
        }
    }
    return false;
}

template<typename K>
bool UBTreeFormula<K>::contains_set(const std::vector<K> &set) const
{
    Node *node = m_root;
    for (unsigned i = 0; i < set.size() && node != NULL; i++) {
        node = lookup_successor(node, set[i]);
    }
    return node != NULL && node->empty();
}

template<typename K>
bool UBTreeFormula<K>::is_cut_by(const Node *node,
                                 const std::vector<K> &set,
                                 unsigned i) const
{
    if (node->empty() || i == set.size()) {
        return false;
    }
    unsigned j;
    for (unsigned k = 0; k < node->size(); k++) {
        j = i;
        while (j < set.size() && set[j] < node->at(k).first) {
            j++;
        }
        if (j == set.size()
            || (set[j] > node->at(k).first
                && !is_cut_by(node->at(k).second, set, j))) {
            return false;
        }
    }
    return true;
}

template<typename K>
size_t UBTreeFormula<K>::count_markers(const Node *node) const
{
    if (node->empty()) {
        return 1;
    } else {
        size_t res = 0;
        for (unsigned k = 0; k < node->size(); k++) {
            res += count_markers(node->at(k).second);
        }
        return res;
    }
}

template<typename K>
void UBTreeFormula<K>::print(const Node *node, std::vector<K> &subset) const
{
    if (node->empty()) {
        std::cout << "[";
        for (unsigned i = 0; i < subset.size(); i++) {
            std::cout << (i > 0 ? ", " : "")
                      << subset[i];
        }
        std::cout << "]" << std::endl;
    } else {
        std::vector<std::pair<K, const Node *> > x;
        for (unsigned k = 0; k < node->size(); k++) {
            x.emplace_back(node->at(k).first, node->at(k).second);
        }
        std::sort(x.begin(), x.end());
        for (unsigned i = 0; i < x.size(); i++) {
            subset.push_back(x[i].first);
            print(x[i].second, subset);
            subset.pop_back();
        }
    }
}

template<typename K>
bool UBTreeFormula<K>::contains_subset_of(const std::vector<K> &set) const
{
    return !m_root->empty() && contains_subset_of(m_root, set, 0);
}

template<typename K>
bool UBTreeFormula<K>::is_cut_by(const std::vector<K> &set) const
{
    return is_cut_by(m_root, set, 0);
}

template<typename K>
std::pair<typename UBTreeFormula<K>::Node*, bool> UBTreeFormula<K>::insert(
    const std::vector<K> &set)
{
    std::pair<Node*, bool> res(NULL, false);
    res.first = insert(m_root, set, 0, res.second);
    return res;
}

template<typename K>
void UBTreeFormula<K>::erase(unsigned, const std::vector<K> &set)
{
    erase(m_root, set, 0);
}

template<typename K>
void UBTreeFormula<K>::remove_all_supersets_of(const std::vector<K> &set)
{
    remove_all_supersets_of(m_root, set, 0);
}

template<typename K>
unsigned UBTreeFormula<K>::delete_element(unsigned,
        const std::vector<K> &set,
        K elem)
{
    Node *node = m_root;

    unsigned i = 0;
    if (!set.empty()) {
        while (set[i] < elem && node != NULL) {
            node = lookup_successor(node, set[i]);
            i++;
        }
    }

    bool found = false;
    bool inserted = false;

    if (node != NULL) {
        unsigned k;
        Node *succ = lookup_successor(node, elem, k);
        if (succ != NULL) {
            if (erase(succ, set, i, found)) {
                delete(succ);
                node->erase(node->begin() + k);
            }
        }
        insert(node, set, i, inserted);
    }

    return inserted;

    // if (!found && !inserted) {
    //     return 0;
    // } else if (!found) {
    //     return 1;
    // } else if (!inserted) {
    //     return 2;
    // } else {
    //     return 3;
    // }
}

template<typename K>
bool UBTreeFormula<K>::insert_element(unsigned,
                                      const std::vector<K> &set,
                                      K elem,
                                      unsigned status)
{
    // if (status == 0) {
    //     return false;
    // }

    Node *node = NULL;
    Node *succ = m_root;
    unsigned i = 0;
    while (set[i] < elem && succ != NULL) {
        node = succ;
        succ = lookup_successor(node, set[i]);
        i++;
    }

    // if (status >= 2) {
    if (succ == NULL) {
        force_insert(node, set, i - 1);
    } else {
        force_insert(succ, set, i);
    }
    // }

    // if ((status == 1 || status == 3) && succ != NULL) {
    if (status && succ != NULL) {
        bool found;
        erase(succ, set, i + 1, found);
    }

    return true;
}

template<typename K>
size_t UBTreeFormula<K>::size() const
{
    return m_root->empty() ? 0 : count_markers(m_root);
}

template<typename K>
void UBTreeFormula<K>::print() const
{
    std::vector<K> subset;
    print(m_root, subset);
}


}

#endif
