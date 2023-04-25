#ifdef SET_UTILS_H

namespace set_utils
{

template<typename T>
void clear(T &x)
{
    T dummy;
    x.swap(dummy);
}

template<typename C, typename T>
bool contains(const C &v, const T &elem)
{
    typename C::const_iterator it =
        lower_bound(v.begin(), v.end(), elem);
    return it != v.end() && elem == (*it);
}

template<typename C>
bool intersects(const C &x, const C &y)
{
    typename C::const_iterator i = x.begin();
    typename C::const_iterator ei = x.end();
    typename C::const_iterator j = y.begin();
    typename C::const_iterator ej = y.end();
    while (i != ei && j != ej) {
        if (*i == *j) {
            return true;
        } else if (*i < *j) {
            i++;
        } else {
            j++;
        }
    }
    return false;
}

template<typename C, typename Less>
bool intersects1(const C &x, const C &y, const Less &less)
{
    typename C::const_iterator i = x.begin();
    typename C::const_iterator ei = x.end();
    typename C::const_iterator j = y.begin();
    typename C::const_iterator ej = y.end();
    while (i != ei && j != ej) {
        if (less(*i, *j)) {
            i++;
        } else if (less(*j, *i)) {
            j++;
        } else {
            return true;
        }
    }
    return false;
}

template<typename C, typename T>
bool intersects(const C &x, const C &y, T &res)
{
    typename C::const_iterator i = x.begin();
    typename C::const_iterator ei = x.end();
    typename C::const_iterator j = y.begin();
    typename C::const_iterator ej = y.end();
    while (i != ei && j != ej) {
        if (*i == *j) {
            res = *i;
            return true;
        } else if (*i < *j) {
            i++;
        } else {
            j++;
        }
    }
    return false;
}

template<typename C>
void negated_intersection(const C &x, const C &y, C &r1, C &r2)
{
    typename C::const_iterator i = x.begin();
    typename C::const_iterator ei = x.end();
    typename C::const_iterator j = y.begin();
    typename C::const_iterator ej = y.end();
    while (i != ei && j != ej) {
        if (*i == *j) {
            i++;
            j++;
        } else if (*i < *j) {
            r1.push_back(*i);
            i++;
        } else {
            r2.push_back(*j);
            j++;
        }
    }
    r1.insert(r1.end(), i, ei);
    r2.insert(r2.end(), j, ej);
}

template<typename C, typename T>
bool insert(C &v, const T &elem)
{
    typename C::iterator it =
        lower_bound(v.begin(), v.end(), elem);
    if (it == v.end() || elem != (*it)) {
        v.insert(it, elem);
        return true;
    }
    return false;
}

template<typename C, typename T, typename Compare>
bool insert(C &v, const T &elem, Compare &compare)
{
    typename C::iterator it =
        lower_bound(v.begin(), v.end(), elem, compare);
    if (it == v.end() || !compare(elem, *it)) {
        v.insert(it, elem);
        return true;
    }
    return false;
}

template<typename C, typename T>
std::pair<typename C::iterator, bool> insert_get_position(
    C &v,
    const T &elem)
{
    typename C::iterator it =
        lower_bound(v.begin(), v.end(), elem);
    bool x = false;
    if (it == v.end() || elem != (*it)) {
        it = v.insert(it, elem);
        x = true;
    }
    return std::pair<typename C::iterator, bool>(it, x);
}

template<typename C, typename T>
bool remove(C &v, const T &elem)
{
    typename C::iterator i =
        lower_bound(v.begin(), v.end(), elem);
    if (i != v.end() && *i == elem) {
        v.erase(i);
        return true;
    }
    return false;
}

template<typename C, typename T>
void remove_lower_bound(C &v, const T &elem)
{
    assert(lower_bound(v.begin(), v.end(), elem) != v.end());
    v.erase(lower_bound(v.begin(), v.end(), elem));
}

template<typename C, typename T, typename Compare>
void remove_lower_bound(C &v, const T &elem, Compare &compare)
{
    assert(lower_bound(v.begin(), v.end(), elem, compare) != v.end());
    v.erase(lower_bound(v.begin(), v.end(), elem, compare));
}

template<typename C>
size_t inplace_union(C &dest, const C &src)
{
    size_t res = 0;
    typename C::const_iterator i = src.begin();
    typename C::const_iterator e = src.end();
    typename C::iterator j = dest.begin();
    while (i != e && j != dest.end()) {
        if (*i < *j) {
            j = dest.insert(j, *i);
            res++;
            i++;
            j++;
        } else if (*i == *j) {
            i++;
            j++;
        } else {
            j++;
        }
    }
    while (i != e) {
        j = dest.insert(j, *i);
        res++;
        i++;
        j++;
    }
    assert(std::is_sorted(dest.begin(), dest.end()));
    assert(std::unique(dest.begin(), dest.end()) == dest.end());
    assert(std::includes(dest.begin(), dest.end(), src.begin(), src.end()));
    return res;
}

template<typename C>
size_t inplace_difference(C &x, const C &y)
{
    size_t res = 0;
    typename C::iterator i = x.begin();
    typename C::const_iterator j = y.begin();
    typename C::const_iterator e = y.end();
    while (i != x.end() && j != e)  {
        if (*i == *j) {
            i = x.erase(i);
            res++;
            j++;
        } else if (*i > *j) {
            j++;
        } else {
            i++;
        }
    }
    assert(!intersects(x, y));
    return res;
}

template<typename C, typename F>
size_t inplace_difference(C &x, const C &y, const F &f)
{
    size_t res = 0;
    typename C::iterator i = x.begin();
    typename C::const_iterator j = y.begin();
    typename C::const_iterator e = y.end();
    while (i != x.end() && j != e)  {
        if (*i == *j) {
            f(*i);
            i = x.erase(i);
            res++;
            j++;
        } else if (*i > *j) {
            j++;
        } else {
            i++;
        }
    }
    assert(!intersects(x, y));
    return res;
}


template<typename C>
void inplace_intersection(C &x, const C &y)
{
    typename C::iterator i = x.begin();
    typename C::iterator iE = x.end();
    typename C::const_iterator j = y.begin();
    typename C::const_iterator jE = y.end();
    typename C::iterator k = x.begin();
    while (i != iE && j != jE) {
        if (*i == *j) {
            if (i != k) {
                std::iter_swap(i, k);
            }
            i++;
            j++;
            k++;
        } else if (*i < *j) {
            i++;
        } else {
            j++;
        }
    }
    x.erase(k, iE);
}


template<typename C>
bool disjoint(const C &x, const C &y)
{
    typename C::const_iterator i = x.begin();
    typename C::const_iterator iE = x.end();
    typename C::const_iterator j = y.begin();
    typename C::const_iterator jE = y.end();
    while (i != iE && j != jE) {
        if (*i == *j) {
            return false;
        } else if (*i < *j) {
            i++;
        } else {
            j++;
        }
    }
    return true;
}

template<typename C>
bool is_set(C& c)
{
    return std::is_sorted(c.begin(), c.end())
        && std::unique(c.begin(), c.end()) == c.end();
}

}

#endif
