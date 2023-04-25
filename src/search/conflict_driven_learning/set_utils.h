#ifndef SET_UTILS_H
#define SET_UTILS_H

#include <algorithm>
#include <utility>
#include <cassert>

namespace set_utils
{

template<typename T>
void clear(T &x);


template<typename C, typename T>
bool contains(const C &v, const T &elem);


template<typename C>
bool intersects(const C &x, const C &y);


template<typename C, typename Less>
bool intersects1(const C &x, const C &y, const Less &less);


template<typename C, typename T>
bool intersects(const C &x, const C &y, T &res);


template<typename C>
void negated_intersection(const C &x, const C &y, C &r1, C &r2);


template<typename C, typename T>
bool insert(C &v, const T &elem);


template<typename C, typename T, typename Compare>
bool insert(C &v, const T &elem, Compare &compare);


template<typename C, typename T>
std::pair<typename C::iterator, bool> insert_get_position(
    C &v,
    const T &elem);


template<typename C, typename T>
bool remove(C &v, const T &elem);


template<typename C, typename T>
void remove_lower_bound(C &v, const T &elem);


template<typename C, typename T, typename Compare>
void remove_lower_bound(C &v, const T &elem, Compare &compare);


template<typename C>
size_t inplace_union(C &dest, const C &src);


template<typename C>
size_t inplace_difference(C &x, const C &y);


template<typename C, typename F>
size_t inplace_difference(C &x, const C &y, const F &f);


template<typename C>
void inplace_intersection(C &dest, const C &src);


template<typename C>
bool disjoint(const C &x, const C &y);

template<typename C>
bool is_set(C& c);

}

#include "set_utils.cc"

#endif
