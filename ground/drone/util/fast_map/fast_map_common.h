// @file drone/util/fast_map/fast_map_common.h
#pragma once

namespace Ground::Drone
{

    using U32 = unsigned int;
    using U64 = unsigned long long;

    template <typename T>
    constexpr T Max(T a, T b)
    {
        return (a > b) ? a : b;
    }

#if defined(__GNUC__) || defined(__clang__)
#define FASTMAP_FORCE_INLINE inline __attribute__((always_inline))
#else
#define FASTMAP_FORCE_INLINE inline
#endif

    // Sentinel value for empty slots (for U32 keys/values)
    constexpr U32 kInvalidU32 = 0xFFFFFFFFU;

// Fixed-size, linear-probing hash table template, optimized for U32->U32, SIMD,
// and cache alignment
#include <immintrin.h> // For AVX2 intrinsics (if available)

    template <U64 kCapacity, U64 kLoadFactor = 80>
    class FastMap32Impl
    {
    public:
        static constexpr U64 kElemsPerBucketBits = 4;
        static constexpr U64 kElemsPerBucket = 1ULL << kElemsPerBucketBits;
        static constexpr U64 kPaddedCap =
            Max(kCapacity * 100 / kLoadFactor, kElemsPerBucket);
        static constexpr U64 kInternalCap =
            1ULL << (64 - __builtin_clzll(kPaddedCap - 1));
        static constexpr U64 kInternalCapMask = kInternalCap - 1;

        // Align to 64 bytes for AVX2
        alignas(64) U32 m_keys[kInternalCap];
        alignas(64) U32 m_values[kInternalCap];
        U64 m_size;

        FastMap32Impl() { Clear(); }

        // SIMD-accelerated insert/find for U32->U32
        FASTMAP_FORCE_INLINE bool Insert(U32 key, U32 value)
        {
            U64 idx = Hash(key) & kInternalCapMask;
            for (U64 probe = 0; probe < kInternalCap; probe += kElemsPerBucket)
            {
                // SIMD probe for empty or matching key
                __m128i keyvec = _mm_set1_epi32(key);
                __m128i emptyvec = _mm_set1_epi32(kInvalidU32);
                __m128i *bucket = (__m128i *)&m_keys[idx];
                int mask = _mm_movemask_ps(
                    _mm_castsi128_ps(_mm_cmpeq_epi32(*bucket, keyvec)));
                if (mask)
                {
                    int offset = __builtin_ctz(mask);
                    m_values[idx + offset] = value;
                    return true;
                }
                mask = _mm_movemask_ps(
                    _mm_castsi128_ps(_mm_cmpeq_epi32(*bucket, emptyvec)));
                if (mask)
                {
                    int offset = __builtin_ctz(mask);
                    if (m_size >= kCapacity)
                        return false;
                    m_keys[idx + offset] = key;
                    m_values[idx + offset] = value;
                    ++m_size;
                    return true;
                }
                idx = (idx + kElemsPerBucket) & kInternalCapMask;
            }
            return false;
        }

        FASTMAP_FORCE_INLINE U32 *Find(U32 key)
        {
            U64 idx = Hash(key) & kInternalCapMask;
            for (U64 probe = 0; probe < kInternalCap; probe += kElemsPerBucket)
            {
                __m128i keyvec = _mm_set1_epi32(key);
                __m128i *bucket = (__m128i *)&m_keys[idx];
                int mask = _mm_movemask_ps(
                    _mm_castsi128_ps(_mm_cmpeq_epi32(*bucket, keyvec)));
                if (mask)
                {
                    int offset = __builtin_ctz(mask);
                    return &m_values[idx + offset];
                }
                // Early out if any empty slot
                __m128i emptyvec = _mm_set1_epi32(kInvalidU32);
                int emptymask = _mm_movemask_ps(
                    _mm_castsi128_ps(_mm_cmpeq_epi32(*bucket, emptyvec)));
                if (emptymask)
                    return nullptr;
                idx = (idx + kElemsPerBucket) & kInternalCapMask;
            }
            return nullptr;
        }

        FASTMAP_FORCE_INLINE const U32 *Find(U32 key) const
        {
            U64 idx = Hash(key) & kInternalCapMask;
            for (U64 probe = 0; probe < kInternalCap; probe += kElemsPerBucket)
            {
                __m128i keyvec = _mm_set1_epi32(key);
                const __m128i *bucket = (const __m128i *)&m_keys[idx];
                int mask = _mm_movemask_ps(
                    _mm_castsi128_ps(_mm_cmpeq_epi32(*bucket, keyvec)));
                if (mask)
                {
                    int offset = __builtin_ctz(mask);
                    return &m_values[idx + offset];
                }
                __m128i emptyvec = _mm_set1_epi32(kInvalidU32);
                int emptymask = _mm_movemask_ps(
                    _mm_castsi128_ps(_mm_cmpeq_epi32(*bucket, emptyvec)));
                if (emptymask)
                    return nullptr;
                idx = (idx + kElemsPerBucket) & kInternalCapMask;
            }
            return nullptr;
        }

        void Clear()
        {
            for (U64 i = 0; i < kInternalCap; ++i)
            {
                m_keys[i] = kInvalidU32;
            }
            m_size = 0;
        }

        U64 Size() const { return m_size; }
        U64 Capacity() const { return kCapacity; }

    private:
        // Fast integer hash for U32
        static FASTMAP_FORCE_INLINE U64 Hash(U32 x)
        {
            x ^= x >> 16;
            x *= 0x85ebca6b;
            x ^= x >> 13;
            x *= 0xc2b2ae35;
            x ^= x >> 16;
            return static_cast<U64>(x);
        }
    };

    // Alias for U32->U32 map, using the SIMD-optimized implementation
    template <U64 kCapacity>
    using FastMap32 = FastMap32Impl<kCapacity>;

} // namespace Ground::Drone
