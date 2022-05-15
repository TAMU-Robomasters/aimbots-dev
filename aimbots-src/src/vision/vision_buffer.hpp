#pragma once

#include "utils/common_types.hpp"

namespace src::vision {

    template <int maxSize, typename T = uint8_t>
    class visionBuffer {
       private:
        T buffer[maxSize];
        int head = 0, tail = 0;
        T end;
        std::pair<T*, int> lastMsg = {nullptr, 0};

       public:
        visionBuffer<maxSize, T>(T end = '\n') : end(end){};
        ~visionBuffer() { delete[] lastMsg.first; }

        T get(size_t index) { return buffer[index]; }

        size_t getHead() { return head; }
        size_t getTail() { return tail; }
        size_t getMaxSize() { return maxSize; }
        std::pair<T*, size_t> getLastMsg() { return lastMsg; }

        bool isFull() { return head == (tail + 1) % maxSize; }
        bool isEmpty() { return tail == head; }

        size_t size() {
            if (tail >= head) {
                return tail - head;
            }
            if (head > tail) {
                return maxSize - head + tail;
            }
            return 0;
        }

        bool enqueue(T item) {
            if (item == end) {
                this->reset();
                return true;
            }
            if (this->isFull()) {
                return true;
            }
            buffer[tail] = item;
            tail = (tail + 1) % maxSize;
            return false;
        }

        T dequeue() {
            if (this->isEmpty()) {
                return 0;
            }
            T item = buffer[head];

            head = (head + 1) % maxSize;
            return item;
        }

        void reset() {
            size_t sz = this->size();  // calculate size of current buffer

            if (lastMsg.first != NULL) delete[] lastMsg.first;

            this->lastMsg.first = new T[sz];  // assign new memory to first ptr
            this->lastMsg.second = sz;        // assign message size to size

            for (size_t i = 0; i < sz; i++) {
                lastMsg.first[i] = this->dequeue();
            }
        }
    };
}