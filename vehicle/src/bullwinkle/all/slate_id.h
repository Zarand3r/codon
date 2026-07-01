/**
 * @author Richard Bao
 *
 * slate_id — the packed Slate element identifier.
 *
 * A slate_element_t is an opaque 64-bit token produced at Slate *build* time
 * and used at run time to reach an element without a pointer (position
 * independence — a token is meaningful on any string). Everything a hot-path
 * load/store needs is packed inline, so resolution is a few shifts + masks and
 * one array-indexed add (see SlateMemory::load_element_*).
 *
 * Layout (bit 63..0):
 *
 *     [ offset:32 ][ index:26 ][ shard:4 ][ w:1 ][ v:1 ]
 *       byte offset  element    slate_     may_   has_
 *       into shard   index      shard_t    write  validator
 *
 *   - offset (32b): byte offset of the value within its shard  (<= 4 GiB/shard)
 *   - index  (26b): element index within the layout            (<= ~67M elems)
 *   - shard   (4b): slate_shard_t (0..6, or shard_invalid=7 to mark un-bound)
 *   - w       (1b): may_write      (a read-only token/view clears this)
 *   - v       (1b): has_validator  (value must go through an on-write validator)
 *
 * Validity / sentinels:
 *   - slate_element_default == 0 is the "no element" sentinel.
 *   - slate_id_buildup is the *only* way to mint a bound id, and it rejects
 *     index 0. So a bound id always has a non-zero index field and is therefore
 *     never 0 — the "index 0 is reserved" invariant is enforced structurally
 *     here, not delegated to the caller (SlatePathMap just never asks for 0).
 *     slate_id_is_valid is thus (id != 0 && shard < num).
 *   - slate_id_build_invalid(count) encodes shard_invalid + a unique counter:
 *     it is never valid, never the default, and disjoint from every bound id.
 */

#ifndef SLATE_ID_H
#define SLATE_ID_H

#include "src/bullwinkle/all/core/drone_types.h"
#include "src/bullwinkle/all/slate_enums.h"

namespace Drone
{
    /** Opaque packed element identifier. */
    typedef UINT64 slate_element_t;

    /** Element index within a layout (26-bit field; index 0 is reserved). */
    typedef UINT32 slate_index_t;

    /** Byte offset of a value within its shard (32-bit field). */
    typedef UINT32 slate_offset_t;

    /** The "no element" sentinel; a bound id is never equal to this. */
    static const slate_element_t slate_element_default = 0;

    namespace slate_id_detail
    {
        enum : unsigned
        {
            kValidatorShift = 0,
            kWriteShift = 1,
            kShardShift = 2,
            kShardBits = 4,
            kIndexShift = 6,
            kIndexBits = 26,
            kOffsetShift = 32,
            kOffsetBits = 32
        };

        static const slate_element_t kShardMask = (slate_element_t(1) << kShardBits) - 1;
        static const slate_element_t kIndexMax = (slate_element_t(1) << kIndexBits) - 1;
        static const slate_element_t kOffsetMax = (slate_element_t(1) << kOffsetBits) - 1;

        /*
         * The fields must tile the 64-bit word exactly and in order, with no
         * overlap or gap. If a width is ever retuned, these fail to compile
         * rather than silently corrupting ids.
         */
        static_assert(kValidatorShift == 0, "validator bit must be bit 0");
        static_assert(kWriteShift == kValidatorShift + 1, "w follows v");
        static_assert(kShardShift == kWriteShift + 1, "shard follows w");
        static_assert(kIndexShift == kShardShift + kShardBits, "index follows shard");
        static_assert(kOffsetShift == kIndexShift + kIndexBits, "offset follows index");
        static_assert(kOffsetShift + kOffsetBits == 64, "fields fill 64 bits");
        static_assert(num_slate_shard_t <= kShardMask, "shards fit the shard field");

        inline slate_element_t shard_field(const slate_element_t id)
        {
            return (id >> kShardShift) & kShardMask;
        }
    } /* namespace slate_id_detail */

    /**
     * Pack a bound element id.
     *
     * @param[out] id    Set to the packed id on success; untouched on failure.
     * @param may_write  Write bit.
     * @param has_validator Validator bit.
     * @param shard      Owning shard (must be a real shard, < num_slate_shard_t).
     * @param offset     Byte offset within the shard (must fit 32 bits).
     * @param index      Element index (must be >= 1 and fit 26 bits).
     *
     * @return True on success. False (no write) if any field is out of range.
     */
    inline bool slate_id_buildup(slate_element_t &id, const bool may_write,
                                 const bool has_validator,
                                 const slate_shard_t shard,
                                 const slate_offset_t offset,
                                 const slate_index_t index)
    {
        using namespace slate_id_detail;
        if (shard >= num_slate_shard_t)
        {
            return false;
        }
        if (index == 0 || index > kIndexMax || offset > kOffsetMax)
        {
            return false;
        }

        id = (slate_element_t(offset) << kOffsetShift) |
             (slate_element_t(index) << kIndexShift) |
             (slate_element_t(shard) << kShardShift) |
             (slate_element_t(may_write ? 1 : 0) << kWriteShift) |
             (slate_element_t(has_validator ? 1 : 0) << kValidatorShift);
        return true;
    }

    /**
     * Unpack the fields a load/store needs. The index is fetched separately via
     * slate_id_index (matching the SlateMemory / Slate call sites).
     */
    inline void slate_id_breakdown(const slate_element_t id, bool &may_write,
                                   bool &has_validator, slate_shard_t &shard,
                                   slate_offset_t &offset)
    {
        using namespace slate_id_detail;
        may_write = (id >> kWriteShift) & 1;
        has_validator = (id >> kValidatorShift) & 1;
        shard = static_cast<slate_shard_t>(shard_field(id));
        offset = static_cast<slate_offset_t>((id >> kOffsetShift) & kOffsetMax);
    }

    inline slate_index_t slate_id_index(const slate_element_t id)
    {
        using namespace slate_id_detail;
        return static_cast<slate_index_t>((id >> kIndexShift) & kIndexMax);
    }

    inline bool slate_id_can_write(const slate_element_t id)
    {
        return (id >> slate_id_detail::kWriteShift) & 1;
    }

    inline bool slate_id_has_validator(const slate_element_t id)
    {
        return (id >> slate_id_detail::kValidatorShift) & 1;
    }

    /** Return a read-only view of `id` (write bit cleared). Idempotent. */
    inline slate_element_t slate_id_ro(const slate_element_t id)
    {
        return id & ~(slate_element_t(1) << slate_id_detail::kWriteShift);
    }

    /**
     * True iff `id` is bound to a real element: it names a real shard and is not
     * the default sentinel. Un-bound (accountant placeholder) ids encode
     * shard_invalid and are rejected here.
     */
    inline bool slate_id_is_valid(const slate_element_t id)
    {
        return id != slate_element_default &&
               slate_id_detail::shard_field(id) < num_slate_shard_t;
    }

    /**
     * Build a unique un-bound id from a monotonic counter (a token the
     * accountant hands out before it is bound to a name). It carries
     * shard_invalid so slate_id_is_valid is false, is never the default, and is
     * disjoint from every bound id.
     */
    inline slate_element_t slate_id_build_invalid(const slate_element_t count)
    {
        using namespace slate_id_detail;
        return (count << kIndexShift) |
               (slate_element_t(shard_invalid) << kShardShift);
    }

} /* end namespace Drone */

#endif /* SLATE_ID_H */
