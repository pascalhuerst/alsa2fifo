/*
 * Copyright (C) 2018 Pascal Huerst <pascal.huerst@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define PROFILE 0

#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <cmath>
#include <cstring>
#include <limits>

template <typename T>
class BlockingCircularBuffer
{
public:
    BlockingCircularBuffer(size_t size) :
        m_size(size),
        m_buffer(new T[m_size]),
        m_itemsRead(0),
        m_itemsWritten(0),
        m_itemsOverwritten(0),
    #if PROFILE
        m_levelMin(0.f),
        m_levelMax(0.f),
    #endif
        m_readIndex(0),
        m_writeIndex(1)
    {
        memset(m_buffer, 0, m_size * sizeof(T));
#if PROFILE
        resetLevel();
#endif
    }

    ~BlockingCircularBuffer()
    {
        if (m_buffer)
            delete[] m_buffer;
    }

    enum Mode {
        Overwrite,
        Block
    };

    void get(T *buffer, unsigned int size)
    {
        std::unique_lock<std::mutex> mlock(m_mutex);
        while (availableToRead() < size) {
            m_condition.wait(mlock);
        }

        m_itemsRead += size;
#if PROFILE
        float level = 100.f - (static_cast<float>(availableToWrite()) / static_cast<float>(m_size) * 100.f);
        if (level < m_levelMin) m_levelMin = level;
        if (level > m_levelMax) m_levelMax = level;
#endif
        for (unsigned int i=0; i<size; i++) {
            m_readIndex++;
            m_readIndex = m_readIndex % m_size;
            buffer[i] = m_buffer[m_readIndex];
        }

        m_condition.notify_one();
    }

    void set(T *buffer, unsigned int size, Mode m)
    {
        std::unique_lock<std::mutex> mlock(m_mutex);
        while (availableToWrite() < size) {
            if (m == Block) {
                m_condition.wait(mlock);
            } else {
                //std::cout << "set: want:" << size << " could: " << availableToWrite() << std::endl;
                m_itemsOverwritten += size - availableToWrite();
                break;
            }
        }

        m_itemsWritten += size;
#if PROFILE
        float level = 100.f - (static_cast<float>(availableToWrite()) / static_cast<float>(m_size) * 100.f);
        if (level < m_levelMin) m_levelMin = level;
        if (level > m_levelMax) m_levelMax = level;
#endif

        for (unsigned int i=0; i<size; i++) {
            m_writeIndex++;
            m_writeIndex = m_writeIndex % m_size;
            m_buffer[m_writeIndex] = buffer[i];
        }

        m_condition.notify_one();
    }

#if PROFILE
    float fillLevelMax() const
    {
        return m_levelMax;
    }

    float fillLevelMin() const
    {
        return m_levelMin;
    }

    void resetLevel()
    {
        m_levelMax = 0;
        m_levelMin = std::numeric_limits<float>::max();
    }
#endif

    unsigned long itemsRead() const
    {
        return m_itemsRead;
    }

    unsigned long itemsWritten() const
    {
        return m_itemsWritten;
    }

    unsigned long itemsOverwritten() const
    {
        return m_itemsOverwritten;
    }

    unsigned int readyToRead()
    {
        std::unique_lock<std::mutex> mlock(m_mutex);
        return availableToRead();
    }

private:
    inline unsigned int availableToRead() const
    {
        return m_size - availableToWrite();
    }

    inline unsigned int availableToWrite() const
    {
        int tmp = m_readIndex - m_writeIndex;
        int distance = tmp < 0 ? -tmp : tmp;

        return m_writeIndex < m_readIndex ? distance : m_size - distance;
    }

private:
    size_t m_size;
    T *m_buffer;

    std::mutex m_mutex;
    std::condition_variable m_condition;

    std::atomic<unsigned long> m_itemsRead;
    std::atomic<unsigned long> m_itemsWritten;

    std::atomic<unsigned long> m_itemsOverwritten;

#if PROFILE
    std::atomic<float> m_levelMax;
    std::atomic<float> m_levelMin;
#endif
    unsigned int m_readIndex;
    unsigned int m_writeIndex;
};
