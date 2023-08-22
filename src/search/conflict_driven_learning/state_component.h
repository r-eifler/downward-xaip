#ifndef STATE_COMPONENT_H
#define STATE_COMPONENT_H

#include "../task_proxy.h"

namespace conflict_driven_learning
{

template<typename Value>
struct Component {
    virtual ~Component() = default;
    virtual const Value& current() = 0;
    virtual void next(unsigned i = 1) = 0;
    virtual bool end() = 0;
    virtual void reset() = 0;
};

template<typename Value>
struct SingletonComponent : public Component<Value> {
private:
    bool ended;
    Value state;
public:
    SingletonComponent(const Value& state);
    virtual const Value &current() override;
    virtual void next(unsigned) override;
    virtual void reset() override;
    virtual bool end() override;
};

template<typename Value, typename Iterator>
struct ComponentIterator : public Component<Value> {
private:
    Iterator m_begin;
    Iterator m_current;
    Iterator m_end;
public:
    ComponentIterator(Iterator i, Iterator end);
    virtual const Value& current() override;
    virtual void next(unsigned i = 1) override;
    virtual void reset() override;
    virtual bool end() override;
};


using StateComponent = Component<State>;
template<typename Iterator>
using StateComponentIterator = ComponentIterator<State, Iterator>;

using SuccessorComponent = Component<std::pair<int, State> >;
template<typename Iterator>
using SuccessorComponentIterator = ComponentIterator<std::pair<int, State>, Iterator>;

}


namespace conflict_driven_learning
{

template<typename Value>
SingletonComponent<Value>::SingletonComponent(const Value& state)
    : ended(false), state(state)
{}

template<typename Value>
const Value& SingletonComponent<Value>::current()
{
    return state;
}

template<typename Value>
void SingletonComponent<Value>::next(unsigned)
{
    ended = true;
}

template<typename Value>
void SingletonComponent<Value>::reset()
{
    ended = false;
}

template<typename Value>
bool SingletonComponent<Value>::end()
{
    return ended;
}

}


namespace conflict_driven_learning
{

template<typename Value, typename Iterator>
ComponentIterator<Value, Iterator>::ComponentIterator(Iterator i, Iterator end)
    : m_begin(i), m_current(i), m_end(end)
{}

template<typename Value, typename Iterator>
const Value& ComponentIterator<Value, Iterator>::current()
{
    return *m_current;
}

template<typename Value, typename Iterator>
void ComponentIterator<Value, Iterator>::next(unsigned i)
{
    while (i-- > 0) {
        m_current++;
    }
}

template<typename Value, typename Iterator>
void ComponentIterator<Value, Iterator>::reset()
{
    m_current = m_begin;
}

template<typename Value, typename Iterator>
bool ComponentIterator<Value, Iterator>::end()
{
    return m_current == m_end;
}

}

#endif
