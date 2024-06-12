#pragma once

#include <array>
#include <queue>

//completely copied from redlob. Yoink.
template<typename T, typename priority_t>
struct PriorityQueue {
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
                std::greater<PQElement>> elements;

    bool empty() const
    {
        return elements.empty();
    }

    void put(T item, priority_t priority)
    {
        elements.emplace(priority, item);
    }

    T get()
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};