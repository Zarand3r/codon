/**
 * @author Richard Bao
 *
 * slate_info — the umbrella header the Slate consumers include for the full
 * element vocabulary: the packed element id (slate_id), the shard/access/
 * permission enums (slate_enums, via slate_id), the stable type id (slate_type),
 * and the fsw assert macros the inline token/accessor code uses.
 *
 * `slate_tokens.h`, `Slate.h`, `SlateBuilder.h`, etc. `#include` this one header.
 *
 * It also defines the `slate_info<T>` value trait — the bridge between the Slate's
 * typed API (load<T>/store<T>/tokens) and its untyped, byte-oriented shard model.
 * The trait tells the byte layer everything it needs to place, read, and move a
 * value of type T in raw shard memory (its typed views, size/alignment, type id,
 * and construct/copy/from_mem operations), and — crucially — gates which types are
 * allowed in the pointer-free, byte-comparable model at all.
 */

#ifndef SLATE_INFO_H
#define SLATE_INFO_H

#include "src/bullwinkle/all/core/fsw.h"
#include "src/bullwinkle/all/slate_id.h"
#include "src/bullwinkle/all/slate_type.h"

#include <new>
#include <type_traits>

namespace Drone
{
    /**
     * Value trait for a Slate element of type T.
     *
     * Primary template: fixed-size, trivially-copyable values (POD scalars/structs,
     * enums) — the common case. Variable-size types (e.g. `Slice<>`) get their own
     * specialization when that type lands; its `size()` will depend on the value,
     * which is why `size()` takes one.
     *
     * Contract (member set + signatures inferred from the Slate/SlateBuilder call
     * sites): typed views R/W/O; size/alignment/type_id; is_valid; and the value
     * operations construct/copy/from_mem.
     */
    template <typename T>
    struct slate_info
    {
        /** Read view: a const reference into shard memory. */
        using R = const T &;
        /** Write view: a mutable reference into shard memory. */
        using W = T &;
        /** Out-parameter view (e.g. get_element_initial_value writes through it). */
        using O = T &;

        /**
         * May a T live in the Slate? It must be trivially copyable (so a shard is a
         * flat, position-independent, byte-comparable image) and not itself a
         * pointer. NOTE: a trivially-copyable struct that *contains* a pointer
         * cannot be rejected generically in C++17 — the no-pointer rule is enforced
         * structurally by only storing vetted types here; this catches the common
         * offenders (std::string/vector/etc. are not trivially copyable).
         */
        static constexpr bool is_valid()
        {
            /* Reject anything address-like: raw pointers, arrays *of* pointers,
             * member/member-function pointers, and nullptr_t — all trivially
             * copyable but not position-independent. (A pointer nested inside a
             * struct still can't be caught generically in C++17 — structural rule
             * + grep gate cover that; see docs/slate-internals.md.) */
            typedef typename std::remove_all_extents<T>::type E;
            return std::is_trivially_copyable<T>::value &&
                   !std::is_pointer<E>::value &&
                   !std::is_member_pointer<E>::value &&
                   !std::is_null_pointer<E>::value;
        }

        /** Bytes this value occupies in a shard. Fixed for the primary template. */
        static size_t size(R) { return sizeof(T); }

        /** Required alignment of the value in a shard. */
        static size_t alignment() { return alignof(T); }

        /** The stable, cross-string type id for T. */
        static slate_type_t type_id() { return slate_type_id<T>(); }

        /** Copy-construct the value into freshly-allocated shard memory. */
        static bool construct(void *mem, R value)
        {
            ::new (mem) T(value);
            return true;
        }

        /** Overwrite an already-live shard value. */
        static bool copy(R value, W dst)
        {
            dst = value;
            return true;
        }

        /** Typed writable view over shard bytes at `mem`. */
        static W from_mem(void *mem) { return *static_cast<T *>(mem); }

        /** Typed read-only view over shard bytes at `mem`. */
        static R from_mem(const void *mem) { return *static_cast<const T *>(mem); }
    };

} /* end namespace Drone */

#endif /* SLATE_INFO_H */
