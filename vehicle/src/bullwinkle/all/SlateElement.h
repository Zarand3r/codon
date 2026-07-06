/**
 * @author Richard Bao
 *
 * SlateElementMetadata — the per-element layout record the Slate builds during the
 * build phase and freezes: everything about an element *except* its live value
 * (which lives in shard memory). One of these is stored per element path in a
 * SlatePathMap; the element's packed slate_element_t is built from its `shard` +
 * `value_offset` + the map-assigned index.
 *
 * Cold: touched only at build time and on diagnostic lookups, never on the hot
 * load/store path (which goes straight to shard bytes via the packed id).
 */

#ifndef SLATE_ELEMENT_H
#define SLATE_ELEMENT_H

#include "src/bullwinkle/all/slate_info.h"

namespace Drone
{
    /** Identifies the subsystem that owns/created an element (build-time bookkeeping). */
    typedef UINT32 slate_subsystem_id_t;

    /**
     * Fixed layout facts for one Slate element. Aggregate of trivial fields (no
     * owning members) so it copies/moves freely into the SlatePathMap.
     */
    struct SlateElementMetadata
    {
        /** Stable type id of the element's value (slate_type_id<T>()). */
        slate_type_t type_id;

        /** Which shard the value lives in. */
        slate_shard_t shard;

        /** Byte offset of the value within its shard. */
        slate_offset_t value_offset;

        /** Byte size of the value in the shard. */
        slate_offset_t value_size;

        /** Externally-visible access policy for the element. */
        slate_elem_access_t access_policy;

        /** Owning subsystem (build-time bookkeeping). */
        slate_subsystem_id_t subsystem_id;

        /** True if this is a view (aliases a region of a parent element). */
        bool is_view_element;

        SlateElementMetadata(const slate_type_t type_id_,
                             const slate_shard_t shard_,
                             const slate_offset_t value_offset_,
                             const slate_offset_t value_size_,
                             const slate_elem_access_t access_policy_,
                             const slate_subsystem_id_t subsystem_id_,
                             const bool is_view_element_)
            : type_id(type_id_), shard(shard_), value_offset(value_offset_),
              value_size(value_size_), access_policy(access_policy_),
              subsystem_id(subsystem_id_), is_view_element(is_view_element_)
        {}
    };

} /* end namespace Drone */

#endif /* SLATE_ELEMENT_H */
