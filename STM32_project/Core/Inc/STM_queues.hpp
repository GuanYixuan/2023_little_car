/*
 * instruction_queue.hpp
 *
 *  Created on: Aug 30, 2023
 */

#ifndef INC_STM_QUEUES_HPP_
#define INC_STM_QUEUES_HPP_

#include <cstring>
#include <stdint.h>

#define INSTRUCTION_LENGTH 8

struct instruction {
    uint8_t data[INSTRUCTION_LENGTH];
};

#define INSTRUCTION_QUEUE_CAPACITY 100
// 指令队列
class instruction_queue {
    private:
        int16_t start, end;
        instruction data[INSTRUCTION_QUEUE_CAPACITY];
    public:
        instruction_queue();

        // 获取指令队列中指令的数量
        uint16_t count() const;
        /**
         * @brief 向队列中添加指令
         * 
         * @param buffer 装有指令的数组起始位置, 会从其中读取`INSTRUCTION_LENGTH`个字节
         * @return bool 添加是否成功(超出队列容量时添加会失败)
         */
        bool push(const uint8_t* const buffer);
        /**
         * @brief 访问队列中第一个指令
         * 
         * @note 在队列为空时调用此方法后果自负
         */
        const instruction* front() const;
        // 弹出队头的指令
        void pop();
};

#define MESSAGE_QUEUE_CAPACITY 1000
// (发给PC的)消息队列
class message_queue {
    private:
        int16_t start, end;
        uint8_t data[MESSAGE_QUEUE_CAPACITY + 1];
        uint8_t func_buffer[256];
    public:
        message_queue();

        // 队列是否为空
        bool empty() const;
        // 队列剩余容量
        uint16_t size_remain() const;
        /**
         * @brief 向队列中添加待发送的消息
         * 
         * @param buffer 待发送的消息, 会读取至`0x00`为止
         * @return bool 添加是否成功(超出队列容量时添加会失败)
         */
        bool append(const uint8_t* const buffer);
        /**
         * @brief 向队列中添加一条消息, 用法同printf
         * 
         * @return bool 添加是否成功(超出队列容量时添加会失败)
         * @note 该函数可能很慢, 请勿在中断处理函数内调用 
         */
        bool wrap_append(const char* format, ...);
        // 获取当前字符串并执行pop操作
        const uint8_t* retrieve();
};

#endif /* INC_STM_QUEUES_HPP_ */
