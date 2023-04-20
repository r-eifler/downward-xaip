#ifndef LAYERED_MAP_H
#define LAYERED_MAP_H

#include "../state_id.h"

#include <deque>
#include <map>

namespace conflict_driven_learning
{

template<typename Key, typename Value>
class LayeredMultiValueMap {
private:
    using Bucket = std::deque<Value>;
    struct Layer {
        unsigned size;
        std::map<Key, Bucket> bucket;
        Layer() : size(0) {}
    };
    using LayeredMap = std::deque<Layer>;

    LayeredMap m_layers;
public:
    LayeredMultiValueMap();
    virtual ~LayeredMultiValueMap() = default;

    void push_layer();
    void pop_layer();
    unsigned num_layers() const;

    void push(const Key& key, const Value& value);
    Value pop_minimum();
    Value pop_maximum();
    unsigned layer_size() const;
};

}

#include "layered_map.cc"

#endif
