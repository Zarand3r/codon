#pragma once

// Small build-phase utilities.
// Contract from present usage (R1 — fill only what is used):
//   - `str_v` / `str_s` / `str_v_v`  string container aliases
//   - `join(const str_v&) -> std::string`  (diagnostic formatting; space-separated)
//   - `MonotonicPool`  a bump arena: `allocate(size, alignment)` + `release()`
//
// `join`'s separator is not pinned by the call sites (it formats a config line into
// an error message); a single space is the sensible default.

#include "src/bullwinkle/all/core/drone_types.h"
#include "src/bullwinkle/all/core/fsw.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <set>
#include <string>
#include <vector>

namespace Drone
{
    using str_v = std::vector<std::string>;
    using str_s = std::set<std::string>;
    using str_v_v = std::vector<std::vector<std::string>>;

    inline std::string join(const str_v &parts)
    {
        std::string out;
        for (std::size_t i = 0; i < parts.size(); ++i)
        {
            if (i != 0)
            {
                out += ' ';
            }
            out += parts[i];
        }
        return out;
    }

    // Monotonic (bump) arena for phase-scoped allocation: hand out aligned bytes by
    // bumping a cursor; never free individually; `release()` frees everything at the
    // phase boundary. Owns its chunks (move-only). This is the data-oriented-design
    // "arena for phase-based temporary allocation" — it is infrastructure (uses the
    // heap for its own chunks), the thing that lets hot paths avoid per-element alloc.
    class MonotonicPool
    {
    public:
        MonotonicPool() = default;
        ~MonotonicPool() { release(); }
        FSW_DISALLOW_COPY_AND_ASSIGN(MonotonicPool);

        // Return `size` bytes aligned to `alignment` (a power of two). Fatal on OOM.
        void *allocate(std::size_t size, std::size_t alignment)
        {
            FswDebugAssert(alignment != 0 && (alignment & (alignment - 1)) == 0);
            if (!m_chunks.empty())
            {
                void *p = try_alloc(m_chunks.back(), size, alignment);
                if (p != nullptr)
                {
                    return p;
                }
            }
            std::size_t cap = size + alignment;
            if (cap < kDefaultChunk)
            {
                cap = kDefaultChunk;
            }
            UINT8 *base = static_cast<UINT8 *>(std::malloc(cap));
            FswAssert(base != nullptr); // OOM is fatal for a build-phase arena
            m_chunks.push_back(Chunk{base, cap, 0});
            void *p = try_alloc(m_chunks.back(), size, alignment);
            FswAssert(p != nullptr); // a chunk sized for size+alignment must fit
            return p;
        }

        void release()
        {
            for (Chunk &c : m_chunks)
            {
                std::free(c.base);
            }
            m_chunks.clear();
        }

    private:
        struct Chunk
        {
            UINT8 *base;
            std::size_t cap;
            std::size_t used;
        };

        // Bump within one chunk; returns nullptr if it doesn't fit. Aligns the
        // absolute address, so any power-of-two alignment is honored.
        static void *try_alloc(Chunk &c, std::size_t size, std::size_t alignment)
        {
            const std::uintptr_t origin = reinterpret_cast<std::uintptr_t>(c.base);
            const std::uintptr_t start = origin + c.used;
            const std::uintptr_t aligned =
                (start + (alignment - 1)) & ~static_cast<std::uintptr_t>(alignment - 1);
            const std::size_t end = static_cast<std::size_t>(aligned - origin) + size;
            if (end > c.cap)
            {
                return nullptr;
            }
            c.used = end;
            return reinterpret_cast<void *>(aligned);
        }

        static constexpr std::size_t kDefaultChunk = 4096;
        std::vector<Chunk> m_chunks;
    };
} // namespace Drone
