/*
 * instruction_queue.cpp
 *
 *  Created on: Aug 30, 2023
 */

#include <STM_queues.hpp>

#include <cstdio>
#include <cstring>
#include <stdarg.h>

#include "stm32f1xx_hal.h"

// instruction_queue

instruction_queue::instruction_queue() : start(0), end(0) {}

inline uint16_t instruction_queue::count() const {
    if (end >= start) return end - start;
    return INSTRUCTION_QUEUE_CAPACITY - (start - end);
}
bool instruction_queue::push(const uint8_t* const buffer) { // 不能inline?
    if (count() >= INSTRUCTION_QUEUE_CAPACITY - 1) return false;

    memcpy(&data[end], buffer, INSTRUCTION_LENGTH);
    end = (end + 1) % INSTRUCTION_QUEUE_CAPACITY;
    return true;
}
const instruction* instruction_queue::front() const { // 不能inline?
    return &data[start];
}
void instruction_queue::pop() { // 不能inline?
    start = (start + 1) % INSTRUCTION_QUEUE_CAPACITY;
}


// message_queue

message_queue::message_queue() : start(0), end(0), data{0} {}

bool message_queue::empty() const {
    return start == end;
}
inline uint16_t message_queue::size_remain() const {
    if (end >= start) return (MESSAGE_QUEUE_CAPACITY - 1) - (end - start);
    return start - end - 1;
}
bool message_queue::append(const uint8_t* const buffer) {
    uint16_t occupation = strlen((char*)buffer) + 1;
    if (size_remain() < occupation) return false;

    if (end + occupation >= MESSAGE_QUEUE_CAPACITY) {
        memcpy(&data[end], buffer, MESSAGE_QUEUE_CAPACITY - end);
        memcpy(data, buffer + (MESSAGE_QUEUE_CAPACITY - end), occupation - (MESSAGE_QUEUE_CAPACITY - end));
    } else memcpy(&data[end], buffer, occupation);
    end = (end + occupation) % MESSAGE_QUEUE_CAPACITY;
    return true;
}
bool message_queue::wrap_append(const char* format, ...) {
	va_list args;
	va_start(args, format);
	vsprintf((char*)func_buffer, format, args);
	va_end(args);

	return append(func_buffer);
}
const uint8_t* message_queue::retrieve() {
    uint16_t len1 = strlen((char*)(data + start));

    if (start + len1 >= MESSAGE_QUEUE_CAPACITY) {
        uint16_t len2 = strlen((char*)data);
        memcpy(func_buffer, data + start, len1); // 不复制\0
        if (len2) memcpy(func_buffer + len1, data, len2);
        else func_buffer[len1] = '\0';

        start = (start + len1 + len2 + 1) % MESSAGE_QUEUE_CAPACITY;
    } else {
        memcpy(func_buffer, data + start, len1 + 1); // +1 为了把 \0 也复制了

        start = (start + len1 + 1) % MESSAGE_QUEUE_CAPACITY;
    }
    return func_buffer;
}

