#pragma once

#include <unordered_map>
#include <vector>
#include <functional>
#include <queue>
#include <list>
#include <iterator>

using std::list;
using std::unordered_map;
using std::pair;
using std::advance;

template<typename T, typename priority_t>
struct PrioQueue {
    list<pair<T, priority_t>> items;
    unordered_map<T, typename list<pair<T, priority_t>>::iterator> item_map;

    bool empty() const {
        return items.empty();
    }

    void put(T item, priority_t priority) {
        auto it = item_map.find(item);
        if (it != item_map.end()) {
            // Element already in queue, update priority if lower
            if (priority < it->second->second) {
                items.erase(it->second);
                it->second = items.insert(find_position(priority), pair<T, priority_t>(item, priority));
            }
        } else {
            // New element, insert in correct position
            item_map[item] = items.insert(find_position(priority), pair<T, priority_t>(item, priority));
        }
    }

    T get() {
        T best_item = items.front().first;
        item_map.erase(best_item);
        items.pop_front();
        return best_item;
    }

private:
    typename list<pair<T, priority_t>>::iterator find_position(priority_t priority) {
        for (auto it = items.begin(); it != items.end(); ++it) {
            if (priority < it->second) {
                return it;
            }
        }
        return items.end();
    }
};
