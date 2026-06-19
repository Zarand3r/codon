/// @file drone/util/fast_map/fast_map32.h
#pragma once

#include "drone/util/fast_map/fast_map_common.h"

namespace Ground::Drone {

    /// U32 -> U32.
    /// FastMap is a family of fast, fixed-size, linear-probing hash tables.
    ///
    /// Element insertion is guaranteed to succeed up to kCapacity elements, and fail thereafter.
    ///
    /// FastMaps never resize, relocate, or erase elements, so iterators, references, and pointers are
    /// guaranteed to remain valid forever, unless the map is Clear()ed.
    template <U64 kCapacity>
    class FastMap32 : public FastMap<U32, U32, kCapacity, 80> {
    public:
        /// Max load factor. Internal storage capacity is calculated such that inserting the maximum
        /// permitted number of elements (kCapacity) does not exceed this load factor, ensuring good
        /// performance.
        static constexpr U64 kLoadFactor = 80;

        /// @{ Elements are stored in aligned blocks of 16 elements, referred to as 'buckets', such that
        /// we can check an entire bucket at once with an aligned vectorized load.
        static constexpr U64 kElemsPerBucketBits = 4;
        static constexpr U64 kElemsPerBucket = 1ULL << kElemsPerBucketBits;
        /// @}

        /// Pad for load factor.
        static constexpr U64 kPaddedCap = Max(kCapacity * 100 / kLoadFactor, kElemsPerBucket);

        /// @{ Final internal capacity is smallest power of 2 that can hold kPaddedCap.
        static constexpr U64 kInternalCap = 1ULL << (64 - __builtin_clzll(kPaddedCap - 1));
        static constexpr U64 kInternalCapMask = kInternalCap - 1;
        /// @}

        using FastMap<U32, U32, kCapacity, 80>::FastMap;  // Inherit constructors
    };
}