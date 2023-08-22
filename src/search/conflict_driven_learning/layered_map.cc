#ifdef LAYERED_MAP_H

#include <cassert>

namespace conflict_driven_learning
{

template<typename Key, typename Value>
LayeredMultiValueMap<Key, Value>::LayeredMultiValueMap()
{
}

template<typename Key, typename Value>
void
LayeredMultiValueMap<Key, Value>::push_layer()
{
    m_layers.emplace_front();
}

template<typename Key, typename Value>
void
LayeredMultiValueMap<Key, Value>::pop_layer()
{
    m_layers.pop_front();
}

template<typename Key, typename Value>
unsigned
LayeredMultiValueMap<Key, Value>::num_layers() const
{
    return m_layers.size();
}

template<typename Key, typename Value>
void
LayeredMultiValueMap<Key, Value>::push(const Key& key, const Value& value)
{
    assert(!m_layers.empty());
    Layer& layer = m_layers.front();
    layer.size++;
    layer.bucket[key].push_front(value);
}

template<typename Key, typename Value>
Value
LayeredMultiValueMap<Key, Value>::pop_minimum()
{
    assert(!m_layers.empty());
    assert(m_layers.front().bucket.size() > 0);
    assert(!m_layers.front().bucket.begin()->second.empty());
    Layer& layer = m_layers.front();
    auto it = layer.bucket.begin();
    Value value = it->second.front();
    it->second.pop_front();
    if (it->second.empty()) {
        layer.bucket.erase(it);
    }
    layer.size--;
    return value;
}

template<typename Key, typename Value>
Value
LayeredMultiValueMap<Key, Value>::pop_maximum()
{
    assert(!m_layers.empty());
    assert(m_layers.front().bucket.size() > 0);
    assert(!m_layers.front().bucket.rbegin()->second.empty());
    Layer& layer = m_layers.front();
    auto it = --(layer.bucket.end());
    Value value = it->second.front();
    it->second.pop_front();
    if (it->second.empty()) {
        layer.bucket.erase(it);
    }
    layer.size--;
    return value;
}

template<typename Key, typename Value>
unsigned
LayeredMultiValueMap<Key, Value>::layer_size() const
{
    assert(!m_layers.empty());
    return m_layers.front().size;
}

}

#endif
