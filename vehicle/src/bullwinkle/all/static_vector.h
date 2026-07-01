#pragma once

// static_vector<T, N>: a fixed-capacity, inline-storage sequence container — the
// repo's "no heap on the hot path" vector (data-oriented-design skill: bounded,
// no hidden allocation). Capacity N is fixed at compile time; elements live inline.
//
// Interface is exactly what present call sites use (R1 — no extension):
//   default ctor (empty) · size() · push_back(const T&) · operator[] (const + mutable).
//
// Storage is a plain inline array, so all N slots are default-constructed at
// construction and elements are copy-assigned in (not placement-new'd). This keeps
// the type trivial and dependency-free; the cost is a narrowed T requirement
// (default-constructible + copy-assignable) and eager construction of unused slots.
// That is acceptable while T is cheap and these are not constructed on the hot path;
// switch to raw aligned storage + placement-new only if a use site needs a
// non-default-constructible T or a profile shows the eager construction matters.
//
// Bounds: push_back overflow is always checked (cheap, once per append, a real
// corruption risk); operator[] is checked only in debug builds (FswDebugAssert) so
// release indexing is zero-overhead like std::vector::operator[].

#include "src/bullwinkle/all/core/fsw.h"

#include <cstddef>
#include <type_traits>

namespace Drone
{
    template <class T, std::size_t N>
    class static_vector
    {
        static_assert(std::is_default_constructible<T>::value,
                      "static_vector<T,N> requires a default-constructible T "
                      "(inline-array storage)");
        static_assert(std::is_copy_assignable<T>::value,
                      "static_vector<T,N> requires a copy-assignable T");

    public:
        static_vector() : m_size(0) {}

        std::size_t size() const { return m_size; }

        void push_back(const T &value)
        {
            FswAssert(m_size < N); // overflow is a programmer error — fail fast
            m_data[m_size] = value;
            ++m_size;
        }

        T &operator[](std::size_t i)
        {
            FswDebugAssert(i < m_size);
            return m_data[i];
        }

        const T &operator[](std::size_t i) const
        {
            FswDebugAssert(i < m_size);
            return m_data[i];
        }

    private:
        T m_data[N];        // inline storage — no heap
        std::size_t m_size; // count of constructed/used slots
    };
} // namespace Drone
