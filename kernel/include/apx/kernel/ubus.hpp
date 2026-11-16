// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <atomic>
#include <array>
#include <cstddef>

namespace apx::kernel {

template<typename T, size_t BufferSize = 8>
class Topic {
public:
    Topic() : m_head(0) {}

    void publish(const T& data) {
        size_t head = m_head.load(std::memory_order_relaxed);
        m_buffer[head % BufferSize] = data;
        m_head.store(head + 1, std::memory_order_release);
    }

    class Subscription {
    public:
        Subscription(const Topic<T, BufferSize>& topic) 
            : m_topic(topic), m_tail(topic.m_head.load(std::memory_order_relaxed)) {}

        bool updated() const {
            return m_tail < m_topic.m_head.load(std::memory_order_acquire);
        }

        T get() {
            size_t head = m_topic.m_head.load(std::memory_order_acquire);
            if (head - m_tail > BufferSize) {
                // overrun
                m_tail = head - BufferSize;
            }
            T data = m_topic.m_buffer[m_tail % BufferSize];
            m_tail++;
            return data;
        }

        T get_latest() {
            size_t head = m_topic.m_head.load(std::memory_order_acquire);
            if (head == 0) {
                return T{}; // or throw if appropriate, but keeping it lock-free and simple
            }
            m_tail = head;
            return m_topic.m_buffer[(head - 1) % BufferSize];
        }

    private:
        const Topic<T, BufferSize>& m_topic;
        size_t m_tail;
    };

    Subscription subscribe() const {
        return Subscription(*this);
    }

private:
    std::array<T, BufferSize> m_buffer;
    std::atomic<size_t> m_head;
};

} // namespace apx::kernel
