#pragma once

#include "utils/common_types.hpp"

namespace src::vision {

    template <int maxSize>
    class visionBuffer {
       private:
        uint8_t buffer[maxSize];
        int head = 0, tail = 0;
        uint8_t end;
        std::pair<uint8_t*, int> lastMsg = {nullptr, 0};

       public:
        visionBuffer<maxSize>(uint8_t end) : end(end){};
        ~visionBuffer() { delete[] lastMsg.first; }

        uint8_t get(size_t index) { return buffer[index]; }

        size_t getHead() { return head; }
        size_t getTail() { return tail; }
        size_t getMaxSize() { return maxSize; }
        std::pair<uint8_t*, size_t> getLastMsg() { return lastMsg; }

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

        bool enqueue(uint8_t item) {
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

        uint8_t dequeue() {
            if (this->isEmpty()) {
                return 0;
            }
            uint8_t item = buffer[head];

            head = (head + 1) % maxSize;
            return item;
        }

        void reset() {
            size_t sz = this->size();  // calculate size of current buffer

            if (lastMsg.first != NULL) delete[] lastMsg.first;

            this->lastMsg.first = new uint8_t[sz];  // assign new memory to first ptr
            this->lastMsg.second = sz;              // assign message size to size

            for (size_t i = 0; i < sz; i++) {
                lastMsg.first[i] = this->dequeue();
            }
        }
    };
}