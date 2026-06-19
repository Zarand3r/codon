#pragma once

// static_vector<T, N>: a fixed-capacity, inline-storage sequence container — the
// repo's "no heap on the hot path" vector (data-oriented-design skill: bounded,
// no hidden allocation). Capacity N is fixed at compile time; elements live inline.
//
// Interface is exactly what present call sites use (R1 — no extension):
//   default ctor (empty) · size() · push_back(const T&) · operator[] (const + mutable).
// Overflow is a broken invariant: fail-fast via SacAssert (the codebase's assert
// doctrine / domain failure policy).

#include "src/bullwinkle/all/core/sac.h"

#include <cstddef>

namespace Drone
{
    template <class T, std::size_t N>
    class static_vector
    {
    public:
        static_vector() : m_size(0) {}

        std::size_t size() const { return m_size; }

        void push_back(const T &value)
        {
            SacAssert(m_size < N); // overflow is a programmer error — fail fast
            m_data[m_size] = value;
            ++m_size;
        }

        T &operator[](std::size_t i)
        {
            SacAssert(i < m_size);
            return m_data[i];
        }

        const T &operator[](std::size_t i) const
        {
            SacAssert(i < m_size);
            return m_data[i];
        }

    private:
        T m_data[N];        // inline storage — no heap
        std::size_t m_size; // count of constructed/used slots
    };
} // namespace Drone
