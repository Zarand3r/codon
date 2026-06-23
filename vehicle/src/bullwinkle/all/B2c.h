#pragma once

// B2c: a borrowed, const, position-independent byte span (pointer + length). The
// unit of raw shard memory passed to hashing/diffing. Non-owning — valid only
// while the referenced buffer lives; never stored across a cycle boundary.
// Contract from usage: default ctor; `B2c(const void*, size_t)`; `.buf()`; `.len()`.

#include <cstddef>

namespace Drone
{
    class B2c
    {
    public:
        B2c() : buf_(nullptr), len_(0) {}
        B2c(const void *buf, size_t len) : buf_(buf), len_(len) {}

        const void *buf() const { return buf_; }
        size_t len() const { return len_; }

    private:
        const void *buf_;
        size_t len_;
    };
} // namespace Drone
