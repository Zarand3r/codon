#pragma once

// B2: a borrowed, mutable byte span (pointer + length); the writable counterpart of
// B2c. Non-owning. Implicitly converts to B2c (mutable view → const view).
// Contract from usage: default ctor; `B2(void*, size_t)`; `.buf()`; `.len()`;
// usable where a `B2c` is expected.

#include "src/bullwinkle/all/B2c.h"

#include <cstddef>

namespace Drone
{
    class B2
    {
    public:
        B2() : buf_(nullptr), len_(0) {}
        B2(void *buf, size_t len) : buf_(buf), len_(len) {}

        void *buf() const { return buf_; }
        size_t len() const { return len_; }

        operator B2c() const { return B2c(buf_, len_); }

    private:
        void *buf_;
        size_t len_;
    };
} // namespace Drone
